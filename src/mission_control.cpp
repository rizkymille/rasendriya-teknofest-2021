#include "ros/ros.h"
#include "rasendriya/Dropzone.h"
#include "mavros_msgs/WaypointReached.h"
#include "mavros_msgs/Waypoint.h"
#include "mavros_msgs/WaypointPush.h"
#include "mavros_msgs/CommandCode.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/NavSatFix.h"
#include <math.h>
#include <list>

// tuneable variable
float focal_length = 2.9 * 1e-6; // in milimeters
float dropping_altitude = 1.5;
int wp2_reach_counter = 0;

float pixel_to_meter = 0.0002645833;
int waypoint_reached = 0;

void waypoint_reached_callback(const mavros_msgs::WaypointReached& wp_reached){
	waypoint_reached = wp_reached.wp_seq;
}

float cam_angle;
float gps_alt, gps_long, gps_lat, gps_hdg;
int x_dz, y_dz;
int mission_flag;

mavros_msgs::WaypointPush waypoint_push;
ros::ServiceClient waypoint_push_client;

void dropzone_target_callback(const rasendriya::Dropzone& dropzone_loc){
	x_dz = dropzone_loc.x_dropzone;
	y_dz = dropzone_loc.y_dropzone;
	cam_angle = dropzone_loc.cAngle;
}

void gps_callback(const sensor_msgs::NavSatFix& gps_data){
	gps_alt = gps_data.altitude;
	gps_long = gps_data.longitude;
	gps_lat = gps_data.latitude;
}

void gps_hdg_callback(const std_msgs::Float64& gps_hdg_data){
	gps_hdg = gps_hdg_data.data;
}

void mission_flag_callback(const std_msgs::Int8& mis_flag){
	int mission_flag = mis_flag.data;
}

void servo_drop_wp(int servo_ch){
	mavros_msgs::Waypoint wp_drop_servo;

	wp_drop_servo.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
	wp_drop_servo.command = mavros_msgs::CommandCode::DO_SET_SERVO;
	wp_drop_servo.is_current = false;
	wp_drop_servo.autocontinue = true;
	wp_drop_servo.param1 = servo_ch;
	wp_drop_servo.param2 = 1100;

	waypoint_push.request.waypoints.push_back(wp_drop_servo);

	if(waypoint_push_client.call(waypoint_push)){
		ROS_INFO("CHANNEL %d PAYLOAD DROPPING WAYPOINT SENT", servo_ch);
	}
	else {
		ROS_INFO("FAILED TO SEND DROPPING WAYPOINT CHANNEL %d PAYLOAD", servo_ch);
	}

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "mission_control");
	ros::NodeHandle nh;

	ros::ServiceClient waypoint_push_client = nh.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");

	ros::Subscriber waypoint_reached_subscriber = nh.subscribe("/mavros/mission/reached", 1, waypoint_reached_callback);
	ros::Subscriber mission_flag_subscriber = nh.subscribe("mission_flag", 1, mission_flag_callback);
	ros::Subscriber dropzone_target_subscriber = nh.subscribe("vision_dropzone", 3, dropzone_target_callback);
	ros::Subscriber gps_coordinate_subscriber = nh.subscribe("/mavros/global_position/global", 1, gps_callback);
	ros::Subscriber gps_hdg_subscriber = nh.subscribe("/mavros/global_position/compass_hdg", 1, gps_hdg_callback);

	ros::Rate rate(30);

	while(ros::ok() && mission_flag != 0){
		ros::spin();

		mavros_msgs::Waypoint wp_nav_drop;
		mavros_msgs::Waypoint wp_repeat;

		// turn on vision node when waypoint 3 has reached
		if(waypoint_reached == 2){
			wp2_reach_counter += wp2_reach_counter;
		}

		// calculate dropzone coordinate with camera
		if ((x_dz && y_dz) > 0){
			// declare constants
			float pi = 3.14159;
			float R_earth = 6378.1*1e3;

			float X_coord = x_dz*pixel_to_meter*gps_alt/focal_length;
			float Y_coord = y_dz*pixel_to_meter*gps_alt/focal_length;
			float r_dist = sqrt(pow(X_coord, 2) + pow(Y_coord, 2));
			float lat_tgt = (180/pi)*asin(sin(gps_lat)*cos(r_dist/R_earth)+cos(gps_lat)*sin(r_dist/R_earth)*cos(gps_hdg+cam_angle));
			float lon_tgt = gps_long + (180/pi)*atan((sin(gps_hdg+cam_angle)*sin(r_dist/R_earth)*cos(gps_lat)/(cos(r_dist/R_earth)-sin(gps_lat)*sin(lat_tgt))));

			// WP 3
			wp_nav_drop.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
			wp_nav_drop.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
			wp_nav_drop.is_current = false;
			wp_nav_drop.autocontinue = true;
			wp_nav_drop.param2 = 1;
			wp_nav_drop.param3 = 0;
			wp_nav_drop.x_lat = lat_tgt;
			wp_nav_drop.y_long = lon_tgt;
			wp_nav_drop.z_alt = dropping_altitude;
			waypoint_push.request.waypoints.push_back(wp_nav_drop);

			if(waypoint_push_client.call(waypoint_push)){
				ROS_INFO("WAYPOINT DROP SENT");
			}
			else {
				ROS_INFO("FAILED TO SEND WAYPOINT DROP");
			}

			// JUMP TO WP 1 AFTER WP 4 WHEN DETECTION SUCESS;
			wp_repeat.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
			wp_repeat.command = mavros_msgs::CommandCode::DO_JUMP;
			wp_repeat.is_current = false;
			wp_repeat.autocontinue = true;
			wp_repeat.param1 = 1;
			wp_repeat.param2 = 2;
			waypoint_push.request.waypoints.push_back(wp_repeat);

			if(waypoint_push_client.call(waypoint_push)){
				ROS_INFO("REPEAT TO WAYPOINT 1");
			}
			else {
				ROS_INFO("FAILED TO REPEAT TO WAYPOINT 1");
			}
		}

		if(wp2_reach_counter == 1) {
			servo_drop_wp(7);
		}
		else if(wp2_reach_counter == 2) {
			servo_drop_wp(8);
		}

		rate.sleep();
	}
	
	return 0;
}

