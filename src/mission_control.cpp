#include "ros/ros.h"

#include "rasendriya/Dropzone.h"

#include "mavros_msgs/WaypointReached.h"
#include "mavros_msgs/Waypoint.h"
#include "mavros_msgs/WaypointPush.h"
#include "mavros_msgs/WaypointList.h"
#include "mavros_msgs/WaypointPull.h"

#include "mavros_msgs/CommandCode.h"
#include "mavros_msgs/Altitude.h"

#include "geometry_msgs/TwistStamped.h"

#include "sensor_msgs/NavSatFix.h"

#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"

#include <math.h>

// REMINDER: wp_num STARTS FROM 0

int waypoint_reached = 0;

void waypoint_reached_callback(const mavros_msgs::WaypointReached& wp_reached){
	waypoint_reached = wp_reached.wp_seq;
}

// CALLBACKS //

int x_pixel, y_pixel;
float alt, gps_long, gps_lat, gps_hdg;
float vel_y;
bool mission_flag;

mavros_msgs::WaypointPush waypoint_push;

void dropzone_target_callback(const rasendriya::Dropzone& dropzone_loc){
	x_pixel = dropzone_loc.x;
	y_pixel = dropzone_loc.y;
}

void gps_callback(const sensor_msgs::NavSatFix& gps_data){
	gps_long = gps_data.longitude;
	gps_lat = gps_data.latitude;
}

void alt_callback(const mavros_msgs::Altitude& alt_data){
	alt = alt_data.relative;
}

void gps_hdg_callback(const std_msgs::Float64& gps_hdg_data){
	gps_hdg = gps_hdg_data.data;
}

void vel_callback(const geometry_msgs::TwistStamped& vel_data){
	vel_y = vel_data.twist.linear.y;
}

void mission_flag_callback(const std_msgs::Bool& mis_flag){
	mission_flag = mis_flag.data;
}

void waypoint_list_callback(const mavros_msgs::WaypointList& wplist) {
	waypoint_push.request.waypoints = wplist.waypoints;
}

/*
	WAYPOINT MODIFIER APIs
*/

void insert_wp(int _wp_num, const mavros_msgs::Waypoint& _wp){	
	waypoint_push.request.waypoints.insert(waypoint_push.request.waypoints.begin() + _wp_num, _wp);
}

void erase_wp(int _wp_num){
	waypoint_push.request.waypoints.erase(waypoint_push.request.waypoints.begin() + _wp_num);
}

void swap_wp(int _wp_num, const mavros_msgs::Waypoint& _wp) {
	erase_wp(_wp_num);
	insert_wp(_wp_num, _wp);
}

void servo_drop_wp(int servo_ch, int wp_drop_num, mavros_msgs::Waypoint& _wp){
	_wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
	_wp.command = mavros_msgs::CommandCode::DO_SET_SERVO;
	_wp.is_current = true;
	_wp.autocontinue = true;
	_wp.param1 = servo_ch;
	_wp.param2 = 1100;

	insert_wp(wp_drop_num, _wp);
}

// mathematical conversion APIs because c++ library doesn't have it. math.h sucks 

float radians(float _deg) {
	return _deg*(3.14159/180);
} 

float degrees(float _rad) {
	return _rad*(180/3.14159);
}

// projectile motion calculator API

float calc_projectile_distance(float _drop_alt, float _drag_coeff) {
	const float gravity = 9.81; // m/s^2
	const float ball_mass = 0.1; // in kg
	
	float y = (vel_y*ball_mass/_drag_coeff)*(1-exp(-(1-(pow(_drag_coeff,2)*_drop_alt)/(pow(ball_mass, 2)*gravity))));
	return y;
}

// coordinate calculator API

void calc_drop_coord(float& _tgt_laty, float& _tgt_lonx, float _drop_offset, int _calc_mode){
	const float pi = 3.14159;
	const float R_earth = 6378.1*1e3;
	
	float hdg = radians(gps_hdg);
	float lat = radians(gps_lat);
	float lon = radians(gps_long);
	float cam_angle, r_dist;

	if(_calc_mode <= 2) {
		const double focal_length_x = 1.3817707364539*1e3; // calibrated result
		const double focal_length_y = 1.3766463458240*1e3; // calibrated result
		//const float pixel_to_mm = 0.2645;

		float X_meter = x_pixel*alt/focal_length_x;
		float Y_meter = y_pixel*alt/focal_length_y;
		
		r_dist = sqrt(pow(X_meter, 2) + pow(Y_meter + _drop_offset, 2));
		cam_angle = radians(atan2(X_meter, Y_meter));
	}
	else {
		cam_angle = radians(180);
		r_dist = _drop_offset;
	}
	
	// using haversine law
	_tgt_laty = degrees(asin(sin(lat)*cos(r_dist/R_earth) + cos(lat)*sin(r_dist/R_earth)*cos(hdg+cam_angle)));
	_tgt_lonx = degrees(lon + atan2(sin(hdg+cam_angle)*sin(r_dist/R_earth)*cos(lat) , (cos(r_dist/R_earth)-sin(lat)*sin(_tgt_laty))));
}

// MAIN FUNCTION //

int main(int argc, char **argv) {
	int mission_repeat_counter = 0;
	int hit_count = 0;

	float tgt_laty, tgt_lonx;

	std_msgs::Bool vision_flag;

	mavros_msgs::WaypointPull waypoint_pull;

	mavros_msgs::Waypoint wp;

	ros::init(argc, argv, "mission_control");
	ros::NodeHandle nh;

	ros::ServiceClient waypoint_push_client = nh.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");

	ros::ServiceClient waypoint_pull_client = nh.serviceClient<mavros_msgs::WaypointPull>("/mavros/mission/pull");

	ros::Subscriber mission_flag_subscriber = nh.subscribe("/rasendriya/mission_flag", 1, mission_flag_callback);

	ros::Subscriber waypoint_list_sub = nh.subscribe("/mavros/mission/waypoints", 1, waypoint_list_callback);

	ros::Subscriber waypoint_reached_sub = nh.subscribe("/mavros/mission/reached", 1, waypoint_reached_callback);
	ros::Subscriber dropzone_target_sub = nh.subscribe("/rasendriya/dropzone", 3, dropzone_target_callback);
	ros::Subscriber gps_coordinate_sub= nh.subscribe("/mavros/global_position/global", 1, gps_callback);
	ros::Subscriber alt_sub = nh.subscribe("/mavros/altitude", 1, alt_callback);
	ros::Subscriber gps_hdg_sub = nh.subscribe("/mavros/global_position/compass_hdg", 1, gps_hdg_callback);
	ros::Subscriber vel_sub = nh.subscribe("/mavros/global_position/gp_vel", 1, vel_callback);

	ros::Publisher vision_flag_publisher = nh.advertise<std_msgs::Bool>("/rasendriya/vision_flag", 1, true);

	ros::Rate rate(25);
	
	// ROS LAUNCH PARAMETERS //
	
	int calc_mode;
	ros::param::get("/rasendriya/calc_mode", calc_mode);
	
	float dropping_offset, dropping_altitude, drag_coeff;
	ros::param::get("/rasendriya/dropping_altitude", dropping_altitude);

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

	while(ros::ok()) {
		
		ROS_INFO_ONCE("Mission program ready");
		
		if(mission_flag) {
			ROS_INFO_ONCE("Mission program started");
			// increase counter if wp3 reached
			if(waypoint_reached == 1){
				++mission_repeat_counter;
			}
			
			// turn on vision node when wp3 has reached
			if(waypoint_reached == 2 && mission_repeat_counter == 1){
				vision_flag.data = true;
				vision_flag_publisher.publish(vision_flag);
			}
			else {
				vision_flag.data = false;
				vision_flag_publisher.publish(vision_flag);
				ROS_INFO_ONCE("Vision program stopped");
			}

			// dropzone found, confirm by wait for hit_point
			if((x_pixel && y_pixel) != 3000){
				++hit_count;
			}
			else {
				--hit_count;
			}

			// dropzone confirmed
			if(hit_count >= 3){
				
				ROS_INFO_ONCE("DROPZONE TARGET ACQUIRED. PROCEED TO EXECUTE DROPPING SEQUENCE");

				if(calc_mode % 2 == 0) {
					ros::param::get("/rasendriya/drag_coefficient", drag_coeff);
					dropping_offset = calc_projectile_distance(dropping_altitude, drag_coeff);
				}
				else {
					ros::param::get("/rasendriya/dropping_offset", dropping_offset);
				}

				calc_drop_coord(tgt_laty, tgt_lonx, dropping_offset, calc_mode);
				
				// send do jump command to WP 1 from WP 4 after dropzone has found as WP 5
				wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
				wp.command = mavros_msgs::CommandCode::DO_JUMP;
				wp.is_current = false;
				wp.autocontinue = true;
				wp.param1 = 1;
				wp.param2 = 2;
				insert_wp(5, wp);

				// send plane attitude waypoint for dropping as WP 3 in WP 2
				wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
				wp.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
				wp.is_current = false;
				wp.autocontinue = true;
				wp.param2 = 1;
				wp.param3 = 0;
				wp.x_lat = tgt_laty;
				wp.y_long = tgt_lonx;
				wp.z_alt = dropping_altitude;
				insert_wp(3, wp);

				// send waypoint to drop front ball
				servo_drop_wp(7, 4, wp);

				if(waypoint_push_client.call(waypoint_push) && waypoint_pull_client.call(waypoint_pull)){
					ROS_INFO("FIRST WAYPOINT NAVIGATION DROP SENT");
				}
				else {
					ROS_WARN("FAILED TO SEND FIRST WAYPOINT NAVIGATION DROP");
				}

				hit_count = 0;
			}

			// send drop back ball
			if(mission_repeat_counter == 3){
				erase_wp(4);
				servo_drop_wp(8, 4, wp);

				if(waypoint_push_client.call(waypoint_push) && waypoint_pull_client.call(waypoint_pull)){
					ROS_INFO("SECOND WAYPOINT NAVIGATION DROP SENT");
				}
				else {
					ROS_WARN("FAILED TO SEND SECOND WAYPOINT NAVIGATION DROP");
				}

				mission_repeat_counter = 0;
			}
		}

		ros::spinOnce();
		rate.sleep();
	}
	
	return 0;
}

