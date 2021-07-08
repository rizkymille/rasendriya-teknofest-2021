#ifndef PLANE
#define PLANE

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Float64.h"
#include "rasendriya/Dropzone.h"
#include "mavros_msgs/Waypoint.h"
#include "mavros_msgs/WaypointPush.h"
#include "mavros_msgs/CommandCode.h"
#include <list>

class Plane {
public:
	Plane();
	~Plane();

	// Plane servo drops
	void servo_drop_wp(int servo_ch);

	// Callbacks
	void dropzone_target_callback(const rasendriya::Dropzone& dropzone_loc);
	void gps_callback(const sensor_msgs::NavSatFix& gps_data);
	void gps_hdg_callback(const std_msgs::Float64& gps_hdg_data);
	
	int _x_target = 0;
	int _y_target = 0;
	float _angle_target = 0; 
	
	float _gps_altitude, _gps_longitude, _gps_latitude, _gps_hdg;

private:
	ros::Subscriber _dropzone_target_subscriber;
	ros::Subscriber _gps_coordinate_subscriber;
	ros::Subscriber _gps_hdg_subscriber;

	ros::ServiceClient _waypoint_push_client;
};

#endif
