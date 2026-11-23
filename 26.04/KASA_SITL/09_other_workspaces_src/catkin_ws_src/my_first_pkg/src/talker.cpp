#include <ros/ros.h> //ros의 라이브러리를 가져온다.
#include <std_msgs/String.h> //문자열 

int main(int argc, char **argv)//메인함수
{

ros::init(argc, argv, "talker");//talker라는 노드 시작
ros::NodeHandle nh;// ROS Master와 통신하기 위한 객체.
// Publisher, Subscriber, Service 등을 생성할 때 사용한다.
ros::Publisher pub =
nh.advertise<std_msgs::String>("chatter",10);//pub = publisher 객체 생성/advertise토픽생성/std_msgs::String 보낼 데이터 타입/ chatter topic이름
ros::Rate loop_rate(1);//1hz(1s당1번)

while (ros::ok())
{

std_msgs::String msg;//메시지 객체 생성

msg.data = "hello ROS!";

pub.publish(msg);//ros통신의시작 chatter를 topic으로 보냄?

ROS_INFO("%s", msg.data.c_str());//터미널출력 publisher가 잘보내는지 확인 코드

ros::spinOnce();//콜백 함수 확인? 실제로는 하는일 x
loop_rate.sleep();//1hz유지?
}

return 0;
}
