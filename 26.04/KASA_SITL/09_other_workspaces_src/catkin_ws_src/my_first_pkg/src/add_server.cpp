#include <ros/ros.h>
#include <my_first_pkg/AddTwoInts.h> //srv파일 불러오기

bool add(my_first_pkg::AddTwoInts::Request &req,my_first_pkg::AddTwoInts::Response &res)
{
res.sum= req.a + req.b;

ROS_INFO("Request : %ld + %ld", req.a, req.b);
ROS_INFO("Response : %ld", res.sum);

return true;
}

int main(int argc, char **argv)
{

ros::init(argc, argv, "add_server");
ros::NodeHandle nh;
ros::ServiceServer service = nh.advertiseService("add_two_ints",add);

ROS_INFO("Add Server Ready.");

ros::spin();

return 0;

}

