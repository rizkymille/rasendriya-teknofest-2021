#include "ros/ros.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/OverrideRCIn.h"
#include "mavros_msgs/RCIn.h"
#include "mavros_msgs/RadioStatus.h"

int RSSI;
int RC_IN_CH3;
int RC_IN_CH3_THRES = 950;
int RSSI_THRES = 10;

void rc_in_callback(const mavros_msgs::RCIn& rcin_data) {
	RC_IN_CH3 = rcin_data.channels[3];
}

void radio_status_callback(const mavros_msgs::RadioStatus& radio_status_data) {
	RSSI = radio_status_data.remrssi;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "failsafe_mode");
	ros::NodeHandle nh;

	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode", 1);

	ros::Publisher rc_override_in_publisher = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 8);
	
	ros::Subscriber radio_status_subscriber = nh.subscribe("/mavros/radio_status", 1, radio_status_callback);
	ros::Subscriber rcin_throttle_subscriber = nh.subscribe("/mavros/rc/in", 1, rc_in_callback);

	ros::Rate rate(20);

	while (ros::ok()) {
		ros::spinOnce();

		if (RSSI < RSSI_THRES || RC_IN_CH3 < RC_IN_CH3_THRES) {
			usleep(3000000);
			
			mavros_msgs::SetMode flight_mode;
			flight_mode.request.base_mode = 88;

			if (set_mode_client.call(flight_mode)) {
				ROS_INFO("OVERRIDING CONTROL. PLANE AT GUIDED FAILSAFE MODE")
			}
			else {
				ROS_INFO("WARNING: FAILED TO OVERRIDE FAILSAFE MODE")
			}

			mavros_msgs::OverrideRCIn rc_in_override_val;
			rc_in_override_val.channels[1] = 1100;
			rc_in_override_val.channels[2] = 1900;
			rc_in_override_val.channels[3] = 65535;
			rc_in_override_val.channels[4] = 1100;
			rc_in_override_val.channels[5] = 65535;
			rc_in_override_val.channels[6] = 65535;
			rc_in_override_val.channels[7] = 1900;
			rc_in_override_val.channels[8] = 1900;
			rc_override_in_publisher.publish(rc_in_override_val)

			break;
		}

		rate.sleep();
	}

	return 0;
}

