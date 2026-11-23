#include <ros/ros.h>
#include <my_first_pkg/Person.h>

int main(int argc , char **argv)
{
ros::init(argc, argv, "person_publisher");
ros::NodeHandle nh;
ros::Publisher pub = nh.advertise<my_first_pkg::Person>("person",10);
ros::Rate loop_rate(1);

while (ros::ok())
{
my_first_pkg::Person msg;

msg.name = "kim";
msg.age = 23;
msg.height = 175.5;

pub.publish(msg);

 ROS_INFO("Send : %s %d %.1f",
                 msg.name.c_str(),
                 msg.age,
                 msg.height);

loop_rate.sleep();
}
return 0;
}
