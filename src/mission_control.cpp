#include "ros/ros.h"
#include "rasendriya/Dropzone.h"
#include "mavros_msgs/WaypointReached.h"
#include "mavros_msgs/Waypoint.h"
#include "mavros_msgs/WaypointPush.h"
#include "mavros_msgs/CommandCode.h"
#include "mavros_msgs/Altitude.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/NavSatFix.h"
#include <math.h>
#include <vector>

// REMINDERS: wp_num STARTS FROM 0

int waypoint_reached = 0;

void waypoint_reached_callback(const mavros_msgs::WaypointReached& wp_reached){
	waypoint_reached = wp_reached.wp_seq;
}

int x_dz, y_dz;
float cam_angle;
float alt, gps_long, gps_lat, gps_hdg;
int mission_flag = 1;

mavros_msgs::WaypointPush waypoint_push;
ros::ServiceClient waypoint_push_client;

// CALLBACKS //

void dropzone_target_callback(const rasendriya::Dropzone& dropzone_loc){
	x_dz = dropzone_loc.x_dropzone;
	y_dz = dropzone_loc.y_dropzone;
	cam_angle = dropzone_loc.cAngle;
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

void mission_flag_callback(const std_msgs::Int8& mis_flag){
	mission_flag = mis_flag.data;
}

// MECHANISMS, CALCULATORS, AND METHODS //

void insert_wp(int _wp_num, mavros_msgs::Waypoint _wp){	
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
		ROS_INFO("FAILED TO SEND DROPPING WAYPOINT CHANNEL %d PAYLOAD", servo_ch);
	}
}

struct targetCoordinate {
	float longitude, latitude;
};

struct targetCoordinate calc_drop_coord(){
	struct targetCoordinate targetCoordinate;
	float pi = 3.14159;
	float R_earth = 6378.1*1e3;
	float pixel_to_meter = 0.0002645833;
	float focal_length = 0.29; // tune value until correct
	float offset_drop = 2;

	float X_coord = x_dz*pixel_to_meter*alt/focal_length;
	float Y_coord = (y_dz + offset_drop)*pixel_to_meter*alt/focal_length;
	float r_dist = sqrt(pow(X_coord, 2) + pow(Y_coord, 2));
	targetCoordinate.latitude = (180/pi)*asin(sin(gps_lat)*cos(r_dist/R_earth)+cos(gps_lat)*sin(r_dist/R_earth)*cos(gps_hdg+cam_angle));
	targetCoordinate.longitude = gps_long + (180/pi)*atan((sin(gps_hdg+cam_angle)*sin(r_dist/R_earth)*cos(gps_lat)/(cos(r_dist/R_earth)-sin(gps_lat)*sin(targetCoordinate.latitude))));
	return targetCoordinate;
}


int main(int argc, char **argv) {
	int mission_repeat_counter = 0;
	int hit_count = 0;

	ros::init(argc, argv, "mission_control");
	ros::NodeHandle nh;

	ros::ServiceClient waypoint_push_client = nh.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");

	ros::Subscriber mission_flag_subscriber = nh.subscribe("mission_flag", 1, mission_flag_callback);

	ros::Subscriber waypoint_reached_subscriber = nh.subscribe("/mavros/mission/reached", 1, waypoint_reached_callback);
	ros::Subscriber dropzone_target_subscriber = nh.subscribe("dropzone_detector", 3, dropzone_target_callback);
	ros::Subscriber gps_coordinate_subscriber = nh.subscribe("/mavros/global_position/global", 1, gps_callback);
	ros::Subscriber alt_subscriber = nh.subscribe("/mavros/altitude", 1, alt_callback);
	ros::Subscriber gps_hdg_subscriber = nh.subscribe("/mavros/global_position/compass_hdg", 1, gps_hdg_callback);

	ros::Publisher vision_flag_publisher = nh.advertise<std_msgs::Int8>("vision_flag", 1);

	ros::Rate rate(30);

	while(ros::ok() && mission_flag != 0){
		ros::spin();

		ROS_INFO("dropzone x coordinate: %d \ndropzone y coordinate: %d \ncoordinate angle: %f", x_dz, y_dz, cam_angle);
		ROS_INFO("plane longitude: %f \nplane latitude: %f \nplane altitude: %f \nplane heading: %f", gps_long, gps_lat, alt, gps_hdg);
		// increase counter if wp3 reached
		if(waypoint_reached == 1){
			++mission_repeat_counter;
		}
		// turn on vision node when wp3 has reached
		if(waypoint_reached == 2 && mission_repeat_counter == 1){
			std_msgs::Int8 vision_flag;
			vision_flag.data = 1;
			vision_flag_publisher.publish(vision_flag);
		}
		else {
			std_msgs::Int8 vision_flag;
			vision_flag.data = -1;
			vision_flag_publisher.publish(vision_flag);
		}

		// dropzone found, confirm by wait for hit_point
		bool found_condition = (x_dz && y_dz) > (0 || NAN);
		if(found_condition){
			++hit_count;
		}
		else {
			hit_count = 0;
		}

		// dropzone confirmed
		if(hit_count >= 3){
			ROS_INFO("DROPZONE TARGET ACQUIRED. PROCEED TO EXECUTE DROPPING SEQUENCE");
			// proceed calculate target coordinate
			targetCoordinate tgt_coord = calc_drop_coord();
			
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
				ROS_INFO("FAILED TO REPEAT MISSION");
			}


			// send plane attitude waypoint for dropping as WP 3 in WP 2
			mavros_msgs::Waypoint wp_nav_drop;
			wp_nav_drop.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
			wp_nav_drop.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
			wp_nav_drop.is_current = false;
			wp_nav_drop.autocontinue = true;
			wp_nav_drop.param2 = 1;
			wp_nav_drop.param3 = 0;
			wp_nav_drop.x_lat = tgt_coord.latitude;
			wp_nav_drop.y_long = tgt_coord.longitude;
			wp_nav_drop.z_alt = 1.5; // dropping altitude. please tune
			insert_wp(3, wp_nav_drop);

			if(waypoint_push_client.call(waypoint_push) && waypoint_push.response.success){
				ROS_INFO("WAYPOINT NAVIGATION DROP SENT");
			}
			else {
				ROS_INFO("FAILED TO SEND WAYPOINT NAVIGATION DROP");
			}

			servo_drop_wp(7,4);

			if(mission_repeat_counter == 3){
				erase_wp(4);
				servo_drop_wp(8,4);
			}
		}

		rate.sleep();
	}
	
	return 0;
}

