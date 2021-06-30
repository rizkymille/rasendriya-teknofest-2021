#include "rasendriya/plane.h"

Plane::Plane(){
	ros::NodeHandle _nh;
	ros::NodeHandle _nh_priv("~");

	_set_front_servo_client = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command", 1);
	_set_back_servo_client = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command", 1);

	_dropzone_target_subscriber = _nh.subscribe("vision_dropzone", 3, &Copter::dropzone_target_callback, this);
	_gps_coordinate_subscriber = _nh.subscribe("/mavros/global_position/global", 3, gps_callback);
	_gps_hdg_subscriber = _nh.subscribe("/mavros/global_position/compass_hdg", 1, gps_hdg_callback);
}

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



// == SERVO DROPS ==

void Plane::front_servo_drop(){
	mavros_msgs::CommandLong do_set_front_servo;
	do_set_front_servo.request.broadcast = true;
	do_set_front_servo.request.command = mavros_msgs::CommandLong::CMD_DO_SET_SERVO;
	do_set_front_servo.request.param1 = 7;
	do_set_front_servo.request.param2 = 1100;

	if(set_front_servo_client.call(do_set_front_servo)){
		ROS_INFO("FRONT SERVO ENGAGED. DROPPING FRONT PAYLOAD");
	}
	else {
		ROS_INFO("FAILED TO ENGAGE FRONT SERVO");
	}
}

void Plane::back_servo_drop(){
	mavros_msgs::CommandLong do_set_back_servo;
	do_set_back_servo.request.broadcast = true;
	do_set_back_servo.request.command = mavros_msgs::CommandLong::CMD_DO_SET_SERVO;
	do_set_back_servo.request.param1 = 8;
	do_set_back_servo.request.param2 = 1100;

	if(set_back_servo_client.call(do_set_back_servo)){
		ROS_INFO("BACK SERVO ENGAGED. DROPPING BACK PAYLOAD");
	}
	else {
		ROS_INFO("FAILED TO ENGAGE BACK SERVO");
	}
}
