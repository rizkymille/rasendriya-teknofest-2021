#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/OverrideRCIn.h"
#include "mavros_msgs/RCIn.h"
#include "mavros_msgs/RadioStatus.h"

int RC_IN_CH3_THRES = 950;
int RSSI_THRES = 10;

void rc_in_callback(const mavros_msgs::RCIn& rcin_data) {
	RC_IN_CH3 = rcin_data.channels[3];
}

void override_rc_out_callback(const mavros_msgs::OverrideRCIn& override_rcin_data) {
	RC_OUT_CH1 = override_rcin_data.channels[1];
	RC_OUT_CH2 = override_rcin_data.channels[2];
	RC_OUT_CH3 = override_rcin_data.channels[3];
	RC_OUT_CH4 = override_rcin_data.channels[4];
}

int main (int argc, char **argv) {
	ros::init(argc, argv, "failsafe_mode");
	ros::NodeHandle nh;

	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

	ros::Publisher rc_overlisher = nh.advertise<std_msgs::Int8>
	ros::Publisher 
	ros::Publisher

	ros::Subscriber rcin_throttle_subscriber = nh.subscribe("/mavros/rc/in", 1, rc_in_callback);

