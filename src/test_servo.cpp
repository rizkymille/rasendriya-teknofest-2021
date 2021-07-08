#include "ros/ros.h"
#include "mavros_msgs/CommandLong.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_servo");
	ros::NodeHandle nh;

	ros::ServiceClient set_servo_client = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");

	ros::Rate rate(30);

	while(ros::ok()){
		ros::spinOnce();

		mavros_msgs::CommandLong do_set_servo;
			
		for(int servo_num = 7; servo_num <= 8; servo_num++){
			usleep(1000000);
			do_set_servo.request.broadcast = true;
			do_set_servo.request.command = 183;
			do_set_servo.request.param1 = servo_num;
			do_set_servo.request.param2 = 1100;

			if (set_servo_client.call(do_set_servo)) {
				ROS_INFO("SERVO %d IS TRIGGERED", servo_num);
			}
			else {
				ROS_INFO("FAILED TO TRIGGER SERVO %d", servo_num);
			}
		}
		
		break;

		rate.sleep();
	}

	return 0;
}

