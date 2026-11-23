#include <ros/ros.h>

int main(int argc, char **argv){

ros::init(argc,argv, "hello_node");

ros::NodeHandle nh;

ros::Rate loop_rate(1);

while (ros::ok())
{
ROS_INFO("Hello ROS!");

ros::spinOnce();
loop_rate.sleep();
}
return 0;
}
