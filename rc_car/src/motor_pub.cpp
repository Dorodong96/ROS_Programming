// ROS Publisher Code
// NODE OBJECT  : motor_pub
// TOPIC NAME   : motor_msg
// MESSAGE TYPE : std_msgs/String
// DESCRIPTION  : 명령에 따른 모터 제어(Forward, BackWard, Stop)

#include "ros/ros.h"
#include "std_msgs/String.h" // std_msgs/String.h 사용(생성하지 않음)

#define FORWARD	"forward"
#define BACKWARD	"backward"
#define STOP		"stop"
#define RIGHT		"right"
#define LEFT		"left"

#define U_PARAMETER	"distance"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_pub");
    ros::NodeHandle nh;

    // 퍼블리셔 선언
    ros::Publisher motor_pub = nh.advertise<std_msgs::String>("/motor_msg", 100);
	
    // 루프주기 설정 (1초 간격 ; 1Hz)
    ros::Rate loop_rate(2);
    std_msgs::String msg;

    int distance = 0;

    while (ros::ok()) {
	   nh.getParam(U_PARAMETER, distance);

    	   if (distance <= 50){
		  ROS_INFO(BACKWARD);
		  msg.data = BACKWARD;
	   }else if(distance <= 100){
		  ROS_INFO(FORWARD);
		  msg.data = FORWARD;
	   }else if(distance > 100){
		  ROS_INFO(STOP);
		  msg.data = STOP;
	   }

	   motor_pub.publish(msg);
	   loop_rate.sleep();
    }

	return 0;
}

