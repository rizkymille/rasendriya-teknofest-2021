#include "ros/ros.h"
#include "rasendriya/Dropzone.h"
#include "mavros_msgs/CommandLong.h"

int x_dz, y_dz;

void dropzone_target_callback(const rasendriya::Dropzone& dropzone_loc){
	x_dz = dropzone_loc.x_dropzone;
	y_dz = dropzone_loc.y_dropzone;
}

bool trigger_servo(int servo_num, const ros::ServiceClient& _svo_client){
	mavros_msgs::CommandLong do_set_servo;
	do_set_servo.request.broadcast = true;
	do_set_servo.request.command = 183;
	do_set_servo.request.param1 = servo_num;
	do_set_servo.request.param2 = 1100;

	if (_svo_client.call(do_set_servo)) {
		ROS_INFO("SERVO %d IS TRIGGERED", servo_num);
		return true;
	}
	else {
		ROS_WARN("FAILED TO TRIGGER SERVO %d", servo_num);
		return false;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_servo");
	ros::NodeHandle nh;

	int hit_count;

	ros::ServiceClient set_servo_client = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command", 1);

	ros::Subscriber dropzone_target_subscriber = nh.subscribe("/rasendriya/dropzone", 3, dropzone_target_callback);

	while(ros::ok()) {
		if((x_dz && y_dz) > 0) {
			++hit_count;
		}
		else {
			--hit_count;
		}

		if(hit_count >= 3) {
			bool payload1_drop = trigger_servo(6, set_servo_client);
			if(payload1_drop) {
				trigger_servo(7, set_servo_client);
			}
		}
		ros::spinOnce();
	}
	return 0;
}