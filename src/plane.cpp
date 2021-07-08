#include "rasendriya/plane.h"

// == CONSTRUCTOR ==

Plane::Plane(){
	ros::NodeHandle _nh;

	_waypoint_push_client = _nh.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");

	_dropzone_target_subscriber = _nh.subscribe("vision_dropzone", 1, &Plane::dropzone_target_callback, this);
	_gps_coordinate_subscriber = _nh.subscribe("/mavros/global_position/global", 1, &Plane::gps_callback, this);
	_gps_hdg_subscriber = _nh.subscribe("/mavros/global_position/compass_hdg", 1, &Plane::gps_hdg_callback, this);
}

// == DESTRUCTOR ==

Plane::~Plane(){
	ROS_INFO("PLANE IS ERASED!");
}

// == CALLBACKS ==

void Plane::dropzone_target_callback(const rasendriya::Dropzone& dropzone_loc){
	_x_target = dropzone_loc.x_dropzone;
	_y_target = dropzone_loc.y_dropzone;
	_angle_target = dropzone_loc.cAngle;
}

void Plane::gps_callback(const sensor_msgs::NavSatFix& gps_data){
	_gps_altitude = gps_data.altitude;
	_gps_longitude = gps_data.longitude;
	_gps_latitude = gps_data.latitude;
}

void Plane::gps_hdg_callback(const std_msgs::Float64& gps_hdg_data){
	_gps_hdg = gps_hdg_data.data;
}

// == SERVO DROP ==

void Plane::servo_drop_wp(int servo_ch){
	mavros_msgs::WaypointPush waypoint_push;
	mavros_msgs::Waypoint wp_drop_servo;

	wp_drop_servo.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
	wp_drop_servo.command = mavros_msgs::CommandCode::DO_SET_SERVO;
	wp_drop_servo.is_current = false;
	wp_drop_servo.autocontinue = true;
	wp_drop_servo.param1 = servo_ch;
	wp_drop_servo.param2 = 1100;

	waypoint_push.request.waypoints.push_back(wp_drop_servo);

	if(_waypoint_push_client.call(waypoint_push)){
		ROS_INFO("CHANNEL %d PAYLOAD DROPPING WAYPOINT SENT", servo_ch);
	}
	else {
		ROS_INFO("FAILED TO SEND DROPPING WAYPOINT CHANNEL %d PAYLOAD", servo_ch);
	}

}