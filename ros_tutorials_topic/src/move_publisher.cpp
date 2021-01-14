#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_publisher");
    ros::NodeHandle nh;

    ros::Publisher ros_tutorial_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist msg;
    
    int count = 0;

    while (ros::ok())
    {
	   double x[] = {2.0, 2.0, 2.0, 0.0, 0.0, 0.0, 2.0, 2.0, 2.0, 0.0};
	   double z[] = {2.0, 2.0, 2.0, 0.0, 0.0, 0.0, 2.0, 2.0, 2.0, 0.0};

	   msg.angular.z = z[count];
	   msg.linear.x = x[count];

	   ROS_INFO("send msg.angular => %f", msg.angular.z);
	   ROS_INFO("send msg.linear => %f", msg.linear.x);

	   ros_tutorial_pub.publish(msg);
	   loop_rate.sleep();

	   if(count == 9)
		  count = 0;
	   else
		  ++count;
    }
    return 0;
}
