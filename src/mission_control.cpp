#include "ros/ros.h"
#include "rasendriya/plane.h"
#include "math.h"
#include "mavros_msgs/WaypointReached.h"
#include "mavros_msgs/Waypoint.h"
#include "mavros_msgs/WaypointPush.h"
#include "mavros_msgs/CommandCode.h"
#include <list>

float focal_length = 2.9 * 1e-6; // in milimeters
float pixel_to_meter = 0.0002645833;
float dropping_altitude = 1.5;
int waypoint_reached = 0;

void waypoint_reached_callback(const mavros_msgs::WaypointReached& wp_reached){
	waypoint_reached = wp_reached.wp_seq;
}

void mission_flag_callback(const std_msgs::Int8& mission_flag) {
	flag_m = mission_flag.data; 
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "mission_control");
	ros::NodeHandle nh;

	ros::ServiceClient waypoint_push_client = nh.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");

	ros::Publisher vision_flag_publisher = nh.advertise<std_msgs::Int8>("vision_flag", 1);

	ros::Subscriber waypoint_reached_subscriber = nh.subscribe("/mavros/mission/reached", 1, waypoint_reached_callback);
	ros::Subscriber mission_flag_subscriber = nh.subscribe("mission_flag", 1, mission_flag_callback);

	ros:Rate rate(30);

	while(ros::ok() && flag_m != -1){
		ros::spinOnce();

		mavros_msgs::WaypointPush waypoint_push;
		mavros_msgs::Waypoint wp_drop;
		mavros_msgs::Waypoint wp_repeat;

		// turn on vision node when waypoint 3 has reached
		if(waypoint_reached == 3) {
			std_msgs::Int8 vision_flag;
			vision_flag.data = 1;
			vision_flag_publisher.publish(vision_flag);
		}
		// start with vision node shutdown
		else {
			std_msgs::Int8 vision_flag;
			vision_flag.data = -1;
			vision_flag_publisher.publish(vision_flag);
		}

		// calculate lat and lon of dropzone
		X = x_dropzone*pixel_to_meter*_gps_altitude/focal_length;
		Y = y_dropzone*pixel_to_meter*_gps_altitude/focal_length;
		r = sqrt(X^2+Y^2);
		R = 6378.1*1e3;
		lat_tgt = asin(sin(_gps_latitude)*cos(r/R)+cos(_gps_latitude)*sin(r/R)*cos(_gps_hdg+cAngle));
		lon_tgt = _gps_longitude + atan((sin(_gps_hdg+cAngle)*sin(r/R)*cos(_gps_latitude)/(cos(r/R)-sin(_gps_latitude)*sin(lat_tgt))));

		// WP 4
		wp_drop.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
		wp_drop.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
		wp_drop.is_current = false;
		wp_drop.autocontinue = true;
		wp_drop.param2 = 1;
		wp_drop.param3 = 0;
		wp_drop.x_lat = lat_tgt;
		wp_drop.y_long = lon_tgt;
		wp_drop.z_alt = dropping_altitude;
		waypoint_push.request.waypoints.push_back(wp_drop);

		// JUMP TO WP 2 AFTER WP 5 WHEN DETECTION SUCESS;
		wp_repeat.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
		wp_repeat.command = mavros_msgs::CommandCode::DO_JUMP;
		wp_repeat.is_current = false;
		wp_repeat.autocontinue = true;
		wp_repeat.param1 = 2;
		wp_repeat.param2 = 2;
		waypoint_push.request.waypoints.push_back(wp_repeat);
	}
}

// pseudocode

/* from vision_dropzone
get x_dropzone
get y_dropzone
get cAngle

from mavros, GPS
get mavros/global_position/compass_hdg (std_msgs/Float64)
hdg
get mavros/global_position/global (sensor_msgs/NavSatFix)
lat, long, alt

calculate lat and lon of dropzone
r = .......
R = 6378.1*1e3
lat_tgt = arcsin(sin(lat_mid)*cos(r/R)+cos(lat_mid)*sin(r/R)*cos(hdg+cAngle))
lon_tgt = lon_mid + arctan((sin(hdg+cAngle)*sin(r/R)*cos(lat_mid)/(cos(r/R)-sin(lat_mid)*sin(lat_tgt)))


