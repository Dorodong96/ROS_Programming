// ROS Publisher Code
// NODE OBJECT  : motor_pub
// TOPIC NAME   : motor_msg
// MESSAGE TYPE : std_msgs/String
// DESCRIPTION  : 명령에 따른 모터 제어(Forward, BackWard, Stop)

#include "ros/ros.h"
#include "std_msgs/String.h"

#define FORWARD	"forward"
#define BACKWARD	"backward"
#define STOP		"stop"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_pub");
    ros::NodeHandle nh;

    // 퍼블리셔 선언
    ros::Publisher motor_pub = nh.advertise<std_msgs::String>("/motor_msg", 100);
	
    // 루프주기 설정 (1초 간격)
    ros::Rate loop_rate(1);
    std_msgs::String msg;

    int count = 10;

    while (ros::ok()) {
    	   if(count % 2 == 0)	msg.data = FORWARD;
	   else if(count % 3 == 0) 	msg.data = BACKWARD;
	   else if(count % 5 == 0)	msg.data = STOP;

	   motor_pub.publish(msg);
	   loop_rate.sleep();

	   count--;
    }

	return 0;
}

