#ifndef PLANE
#define PLANE

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Float64.h"
#include "rasendriya/Dropzone.h"

class Plane {
public:
	Plane();
	~Plane();

	// Plane servo drops
	void front_servo_drop();
	void back_servo_drop();

	// Callbacks
	void dropzone_target_callback(const rasendriya::Dropzone& dropzone_loc);
	void gps_callback(const sensor_msgs::NavSatFix& gps_data);
	void gps_hdg_callback(const std_msgs::Float64& gps_hdg_data);

private:
	ros::Subscriber _dropzone_target_subscriber;
	ros::Subscriber _gps_coordinate_subscriber;
	ros::Subscriber _gps_hdg_subscriber;

	ros::ServiceClient _set_front_servo_client;
	ros::ServiceClient _set_back_servo_client;

	int _x_target = 0;
	int _y_target = 0;
	float _angle_target = 0; 
};

#endif
