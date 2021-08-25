#include "ros/ros.h"
#include "mavros_msgs/WaypointPush.h"
#include "mavros_msgs/Waypoint.h"
#include "mavros_msgs/WaypointPull.h"
#include "mavros_msgs/WaypointList.h"
#include "mavros_msgs/CommandCode.h"

mavros_msgs::WaypointPush waypoint_push;

void wait(int _time) {
	for(int counter = 0; counter < _time; counter++) {
		ros::spinOnce();
		sleep(1);
	}
}

/*
	WAYPOINT MODIFIER APIs
*/

void add_below_wp(const mavros_msgs::Waypoint& _wp) {
	waypoint_push.request.waypoints.push_back(_wp);
}

void insert_wp(int _wp_num, const mavros_msgs::Waypoint& _wp){	
	waypoint_push.request.waypoints.insert(waypoint_push.request.waypoints.begin() + _wp_num, _wp);
}

void erase_wp(int _wp_num){
	waypoint_push.request.waypoints.erase(waypoint_push.request.waypoints.begin() + _wp_num);
}

// CALLBACKS //

void waypoint_list_callback(const mavros_msgs::WaypointList& wplist) {
	waypoint_push.request.waypoints = wplist.waypoints;
}

// MAIN PROGRAMS //

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_waypoint");
	ros::NodeHandle nh;

	mavros_msgs::Waypoint wp;

	mavros_msgs::WaypointPull waypoint_pull;

	ros::ServiceClient waypoint_push_client = nh.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");

	ros::ServiceClient waypoint_pull_client = nh.serviceClient<mavros_msgs::WaypointPull>("/mavros/mission/pull");
	
	ros::Subscriber waypoint_list_sub = nh.subscribe("/mavros/mission/waypoints", 1, waypoint_list_callback);

	ros::Rate rate(10);

	// first WP loading from FCU. Ensures that companion computer has the same waypoints as FCU
	while(ros::ok()) {
		if(waypoint_push.request.waypoints.size() != 0) {
			ROS_INFO("Number of loaded waypoints: %d", int(waypoint_push.request.waypoints.size()));
			ROS_INFO("Waypoint load from FCU Completed");
			break;
		}
		ros::spinOnce();
		rate.sleep();
	}

	// add WP test
	wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
	wp.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
	wp.is_current = false;
	wp.autocontinue = true;
	wp.param2 = 1;
	wp.param3 = 0;
	wp.x_lat = 2;
	wp.y_long = 2;
	wp.z_alt = 5;
	add_below_wp(wp);
	ROS_INFO("NAV DROPPING INSERTED");

	wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
	wp.command = mavros_msgs::CommandCode::DO_JUMP;
	wp.is_current = false;
	wp.autocontinue = true;
	wp.param1 = 1;
	wp.param2 = 2;
	add_below_wp(wp);
	ROS_INFO("DO_JUMP INSERTED");

	wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
	wp.command = mavros_msgs::CommandCode::DO_SET_SERVO;
	wp.is_current = false;
	wp.autocontinue = true;
	wp.param1 = 7;
	wp.param2 = 1100;
	add_below_wp(wp);
	ROS_INFO("SERVO ACTIVATION INSERTED");

	if(waypoint_push_client.call(waypoint_push) && waypoint_pull_client.call(waypoint_pull)){
		ROS_INFO("FIRST WAYPOINTS SENT!");
	}
	else {
		ROS_WARN("FAILED TO SEND FIRST WAYPOINTS");
	}

	wait(10);

	erase_wp(4);

	wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
	wp.command = mavros_msgs::CommandCode::DO_SET_SERVO;
	wp.is_current = false;
	wp.autocontinue = true;
	wp.param1 = 8;
	wp.param2 = 1100;
	insert_wp(3, wp);

	if(waypoint_push_client.call(waypoint_push) && waypoint_pull_client.call(waypoint_pull)){
		ROS_INFO("SECOND DROPPING WAYPOINTS SENT!");
	}
	else {
		ROS_WARN("FAILED TO SEND FIRST DROPPING WAYPOINTS");
	}

	return 0;
}
