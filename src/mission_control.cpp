#include "ros/ros.h"

#include "rasendriya/Dropzone.h"

#include "mavros_msgs/WaypointReached.h"
#include "mavros_msgs/Waypoint.h"
#include "mavros_msgs/WaypointPush.h"
#include "mavros_msgs/CommandCode.h"
#include "mavros_msgs/Altitude.h"

#include "geometry_msgs/TwistStamped.h"

#include "sensor_msgs/NavSatFix.h"

#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"

#include <math.h>
#include <vector>

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

// MECHANISMS, CALCULATORS, AND METHODS //

mavros_msgs::WaypointPush waypoint_push;
ros::ServiceClient waypoint_push_client;

void insert_wp(int _wp_num, const mavros_msgs::Waypoint& _wp){	
	waypoint_push.request.waypoints.insert(waypoint_push.request.waypoints.begin() + _wp_num, _wp);
}

void erase_wp(int _wp_num){
	waypoint_push.request.waypoints.erase(waypoint_push.request.waypoints.begin() + _wp_num);
}

void servo_drop_wp(int servo_ch, int wp_drop_num){
	mavros_msgs::Waypoint wp_drop_servo;

	wp_drop_servo.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
	wp_drop_servo.command = mavros_msgs::CommandCode::DO_SET_SERVO;
	wp_drop_servo.is_current = true;
	wp_drop_servo.autocontinue = true;
	wp_drop_servo.param1 = servo_ch;
	wp_drop_servo.param2 = 1100;

	insert_wp(wp_drop_num, wp_drop_servo);

	if(waypoint_push_client.call(waypoint_push)){
		ROS_INFO("CHANNEL %d PAYLOAD DROPPING WAYPOINT SENT", servo_ch);
	}
	else {
		ROS_WARN("FAILED TO SEND DROPPING WAYPOINT CHANNEL %d PAYLOAD", servo_ch);
	}
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
	float gravity = 9.81; // m/s^2
	float ball_mass = 0.1; // in kg
	
	float y = (vel_y*ball_mass/_drag_coeff)*(1-exp(-(1-(pow(_drag_coeff,2)*_drop_alt)/(pow(ball_mass, 2)*gravity))));
	return y;
}

// coordinate calculator API

void calc_drop_coord(float& _tgt_laty, float& _tgt_lonx, float _drop_offset){
	
	float focal_length_x = 3; // units in mm. need to calibrate
	float focal_length_y = 3; // units in mm. need to calibrate
	float pixel_to_mm = 0.2645; // in mm

	float X_meter = x_pixel*pixel_to_mm*alt/focal_length_x;
	float Y_meter = y_pixel*pixel_to_mm*alt/focal_length_y;
	
	float pi = 3.14159;
	float R_earth = 6378.1*1e3;

	float hdg = radians(gps_hdg);
	float lat = radians(gps_lat);
	float lon = radians(gps_long);
	
	float r_dist = sqrt(pow(X_meter, 2) + pow(Y_meter + _drop_offset, 2));
	float cam_angle = radians(atan2(X_meter, Y_meter));
	
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

	ros::init(argc, argv, "mission_control");
	ros::NodeHandle nh;

	waypoint_push_client = nh.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");

	ros::Subscriber mission_flag_subscriber = nh.subscribe("/rasendriya/mission_flag", 1, mission_flag_callback);

	ros::Subscriber waypoint_reached_sub = nh.subscribe("/mavros/mission/reached", 1, waypoint_reached_callback);
	ros::Subscriber dropzone_target_sub = nh.subscribe("/rasendriya/dropzone", 3, dropzone_target_callback);
	ros::Subscriber gps_coordinate_sub= nh.subscribe("/mavros/global_position/global", 1, gps_callback);
	ros::Subscriber alt_sub = nh.subscribe("/mavros/altitude", 1, alt_callback);
	ros::Subscriber gps_hdg_sub = nh.subscribe("/mavros/global_position/compass_hdg", 1, gps_hdg_callback);
	ros::Subscriber vel_sub = nh.subscribe("/mavros/global_position/gp_vel", 1, vel_callback);

	ros::Publisher vision_flag_publisher = nh.advertise<std_msgs::Bool>("/rasendriya/vision_flag", 1, true);

	ros::Rate rate(25);
	
	// ROS LAUNCH PARAMETERS
	
	int calc_mode;
	ros::param::get("/rasendriya/calc_mode", calc_mode);
	
	float dropping_offset, dropping_altitude, drag_coeff;
	if(calc_mode == 1) {
		ros::param::get("/rasendriya/dropping_offset", dropping_offset);
	}
	ros::param::get("/rasendriya/dropping_altitude", dropping_altitude);
	ros::param::get("/rasendriya/drag_coefficient", drag_coeff);

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
				
				if(calc_mode == 2) {
					dropping_offset = calc_projectile_distance(dropping_altitude, drag_coeff);
				}
				
				// calculate target coordinate
				calc_drop_coord(tgt_laty, tgt_lonx, dropping_offset);
				
				// send do jump command to WP 1 from WP 4 after dropzone has found as WP 5
				mavros_msgs::Waypoint wp_repeat;
				wp_repeat.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
				wp_repeat.command = mavros_msgs::CommandCode::DO_JUMP;
				wp_repeat.is_current = false;
				wp_repeat.autocontinue = true;
				wp_repeat.param1 = 1;
				wp_repeat.param2 = 2;
				insert_wp(5, wp_repeat);

				if(waypoint_push_client.call(waypoint_push) && waypoint_push.response.success){
					ROS_INFO("DO JUMP TO WAYPOINT 1 SUCCESS");
				}
				else {
					ROS_WARN("FAILED TO REPEAT MISSION");
				}


				// send plane attitude waypoint for dropping as WP 3 in WP 2
				mavros_msgs::Waypoint wp_nav_drop;

				wp_nav_drop.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;

				wp_nav_drop.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
				wp_nav_drop.is_current = false;
				wp_nav_drop.autocontinue = true;
				wp_nav_drop.param2 = 1;
				wp_nav_drop.param3 = 0;
				wp_nav_drop.x_lat = tgt_laty;
				wp_nav_drop.y_long = tgt_lonx;
				wp_nav_drop.z_alt = dropping_altitude;
				insert_wp(3, wp_nav_drop);

				if(waypoint_push_client.call(waypoint_push) && waypoint_push.response.success){
					ROS_INFO("WAYPOINT NAVIGATION DROP SENT");
				}
				else {
					ROS_WARN("FAILED TO SEND WAYPOINT NAVIGATION DROP");
				}
				
				// send waypoint to drop front ball
				servo_drop_wp(7,4);
				hit_count = 0;
			}

			// send drop back ball
			if(mission_repeat_counter == 3){
				erase_wp(4);
				servo_drop_wp(8,4);
				mission_repeat_counter = 0;
			}
		}

		ros::spinOnce();
		rate.sleep();
	}
	
	return 0;
}

