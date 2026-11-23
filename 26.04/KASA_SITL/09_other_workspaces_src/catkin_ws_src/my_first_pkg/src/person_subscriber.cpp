#include <ros/ros.h>
#include <my_first_pkg/Person.h>

void personCallback(const my_first_pkg::Person::ConstPtr& msg)
{
ROS_INFO("Name : %s",msg->name.c_str());
ROS_INFO("Age : %d",msg->age);
ROS_INFO("Height: %.1f",msg->height);
}

int main(int argc,char **argv)
{
ros::init(argc,argv,"person_subscriber");
ros::NodeHandle nh;
ros::Subscriber sub =
 nh.subscribe("person",10,personCallback);
ros::spin();
return 0;
}
