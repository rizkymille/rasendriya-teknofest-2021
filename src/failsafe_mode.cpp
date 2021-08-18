#include "ros/ros.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/OverrideRCIn.h"
#include "mavros_msgs/RCIn.h"
#include "mavros_msgs/RadioStatus.h"
#include "mavros_msgs/CommandBool.h"
#include "std_msgs/Bool.h"

// TIMER CALLBACKS //
bool fs_engage = false;

void short_fs_timer_callback(const ros::TimerEvent& short_event) {
	fs_engage = true;
}

void long_fs_timer_callback(const ros::TimerEvent& long_event) {
	fs_engage = true;
}


int RSSI;
int RC_IN_THR;

void rc_in_callback(const mavros_msgs::RCIn& rcin_data) {
	RC_IN_THR = rcin_data.channels[2];
}

void radio_status_callback(const mavros_msgs::RadioStatus& radio_signal_data) {
	RSSI = radio_signal_data.remrssi;
}

int main(int argc, char **argv) {
	float FS_SHORT_TIMEOUT = 1.5;
	float FS_LONG_TIMEOUT = 40;

	ros::init(argc, argv, "failsafe_mode");
	ros::NodeHandle nh;

	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
	ros::ServiceClient set_arm_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

	ros::Publisher vision_flag_publisher = nh.advertise<std_msgs::Bool>("/rasendriya/vision_flag", 1, true);
	ros::Publisher mission_flag_publisher = nh.advertise<std_msgs::Bool>("/rasendriya/mission_flag", 1, true);
	ros::Publisher control_srf_publisher = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);

	ros::Subscriber radio_status_subscriber = nh.subscribe("/mavros/radio_status", 1, radio_status_callback);
	ros::Subscriber rcin_throttle_subscriber = nh.subscribe("/mavros/rc/in", 1, rc_in_callback);

	ros::Rate rate(10);

	bool change_mode_flag = false;

	// create timers for failsafe
	ros::Timer short_fs_timer = nh.createTimer(ros::Duration(FS_SHORT_TIMEOUT), short_fs_timer_callback);
	ros::Timer long_fs_timer = nh.createTimer(ros::Duration(FS_LONG_TIMEOUT), long_fs_timer_callback);

	ROS_INFO("Rasendriya package launched");

	while(ros::ok()){
		
		ROS_INFO_ONCE("Failsafe mode ready");

		while(!fs_engage) {
			if(RC_IN_THR < 950) {
				short_fs_timer.setPeriod(ros::Duration(FS_SHORT_TIMEOUT));
				short_fs_timer.start();
			}
			else if(RSSI == 0) {
				long_fs_timer.setPeriod(ros::Duration(FS_LONG_TIMEOUT));
				long_fs_timer.start();
			}
			else {
				short_fs_timer.stop();
				long_fs_timer.stop();
				fs_engage = false;
			}
			ros::spinOnce();
			rate.sleep();
		}

		if(!change_mode_flag) {
			// set MANUAL mode
			mavros_msgs::SetMode flight_mode;
			flight_mode.request.custom_mode = "MANUAL";

			// DISARM
			mavros_msgs::CommandBool arm_mode;
			arm_mode.request.value = 0;

			if (set_mode_client.call(flight_mode) && set_arm_client.call(arm_mode)) {
				ROS_WARN("OVERRIDING CONTROL. PLANE AT FAILSAFE MODE");
				sleep(2);
			}
			else {
				ROS_ERROR("WARNING: FAILED TO OVERRIDE FAILSAFE MODE");
				sleep(2);
			}
			change_mode_flag = true;
		}

		// FAILSAFE CONTROL SURFACE
		mavros_msgs::OverrideRCIn control_srf_fs;
		control_srf_fs.channels[0] = 1900; // SERVO 1 AIL
		control_srf_fs.channels[1] = 1900; // SERVO 2 ELE
		control_srf_fs.channels[3] = 1900; // SERVO 4 RUD
		control_srf_publisher.publish(control_srf_fs);

		// shut down vision_dropzone.py and mission_control.cpp
		std_msgs::Bool vision_flag;
		vision_flag.data = false;
		vision_flag_publisher.publish(vision_flag);

		std_msgs::Bool mission_flag;
		mission_flag.data = false;
		mission_flag_publisher.publish(mission_flag);

		ros::spinOnce();
		rate.sleep();
	}
	
	return 0;
}
