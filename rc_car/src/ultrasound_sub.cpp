/* 
* ROS Serial Subsrciber Code
* NODE OBJECT  : ultrasound_sub
* TOPIC NAME   : ultrasound_msg
* MESSAGE TYPE : sensor_msgs/Range
* DESCRIPTION  : 측정된 거리 값 전달 받음
*/

#include <ros/ros.h>
#include <sensor_msgs/Range.h>

#define U_TOPIC_NAME	"ultrasound_msg"
#define U_PARAMETER		"distance"
#define DEBUG			true

// 파라미터 전역변수
int gDis = 0;

// 수신 처리 Callback함수 선언
void msgCallback(const sensor_msgs::Range& range_msg)
{
    ros::NodeHandle nh;

    // range_msg.range == (constptr 선언시) range_msg->range
    int dis = int(range_msg.range*100); 

    // 파라미터 값 업데이트
    nh.setParam(U_PARAMETER, dis);

    if (DEBUG){
     ROS_INFO("getParam() => %d, gDis => %d", nh.getParam(U_PARAMETER, gDis), gDis);
    	ROS_INFO("distance = %dcm", dis);
    }
}

// 엔트리 포인트 함수
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ultrasound_sub");
    ros::NodeHandle nh;

    // 파라미터 초기화
    nh.setParam(U_PARAMETER, 0);

    ros::Subscriber ultrasound_sub = nh.subscribe(U_TOPIC_NAME, 100, msgCallback);
 
    ros::spin();

    return 0;
}

