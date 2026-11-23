#include <ros/ros.h>

int main(int argc, char **argv)
{
ros::init(argc, argv, "parameter_example");
ros::NodeHandle nh;

int number;

nh.getParam("number",number);
ROS_INFO("number = %d",number);

return 0;
}
