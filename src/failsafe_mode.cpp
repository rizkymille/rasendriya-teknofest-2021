#include "ros/ros.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/OverrideRCIn.h"
#include "mavros_msgs/RCIn.h"
#include "mavros_msgs/RadioStatus.h"
#include "mavros_msgs/CommandBool.h"
#include "std_msgs/Int8.h"

int RSSI;
int RC_IN_THR;

void rc_in_callback(const mavros_msgs::RCIn& rcin_data) {
	RC_IN_THR = rcin_data.channels[2];
}

void radio_status_callback(const mavros_msgs::RadioStatus& radio_signal_data) {
	RSSI = radio_signal_data.remrssi;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "failsafe_mode");
	ros::NodeHandle nh;

	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
	ros::ServiceClient set_arm_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

	ros::Publisher vision_flag_publisher = nh.advertise<std_msgs::Int8>("/rasendriya/vision_flag", 1);
	ros::Publisher mission_flag_publisher = nh.advertise<std_msgs::Int8>("/rasendriya/mission_flag", 1);
	ros::Publisher control_srf_publisher = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);

	ros::Subscriber radio_status_subscriber = nh.subscribe("/mavros/radio_status", 1, radio_status_callback);
	ros::Subscriber rcin_throttle_subscriber = nh.subscribe("/mavros/rc/in", 1, rc_in_callback);

	ros::Rate rate(5);

	bool fs_engage;

	while(ros::ok()){
		if(RC_IN_THR < 950) {
			sleep(1.5); // FS_SHORT_TIMEOUT
			fs_engage = true;
		}
		else if(RSSI == 0) {
			sleep(40); // FS_LONG_TIMEOUT
			fs_engage = true;
		}
		else {
			fs_engage = false;
		}

		while(fs_engage) {
			// set GUIDED mode
			mavros_msgs::SetMode flight_mode;
			flight_mode.request.custom_mode = "MANUAL";

			// DISARM
			mavros_msgs::CommandBool arm_mode;
			arm_mode.request.value = 0;

			// FAILSAFE CONTROL SURFACE
			mavros_msgs::OverrideRCIn control_srf_fs;
			control_srf_fs.channels[0] = 1900; // SERVO 1 AIL
			control_srf_fs.channels[1] = 1900; // SERVO 2 ELE
			control_srf_fs.channels[3] = 1900; // SERVO 4 RUD
			control_srf_publisher.publish(control_srf_fs);

			if (set_mode_client.call(flight_mode) && set_arm_client.call(arm_mode)) {
				ROS_INFO("OVERRIDING CONTROL. PLANE AT FAILSAFE MODE");
				sleep(2);
			}
			else {
				ROS_WARN("WARNING: FAILED TO OVERRIDE FAILSAFE MODE");
				sleep(2);
			}

			// shut down vision_dropzone.py and mission_control.cpp
			std_msgs::Int8 vision_flag;
			vision_flag.data = -1;
			vision_flag_publisher.publish(vision_flag);

			std_msgs::Int8 mission_flag;
			mission_flag.data = -1;
			mission_flag_publisher.publish(mission_flag);
		}
		ros::spinOnce();
		rate.sleep();
	}
	
	return 0;
}
