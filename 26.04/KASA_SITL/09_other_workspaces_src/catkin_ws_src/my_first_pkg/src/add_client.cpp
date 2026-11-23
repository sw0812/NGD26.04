#include <ros/ros.h>
#include <my_first_pkg/AddTwoInts.h>
#include <iostream>

int main(int argc, char **argv)
{

ros::init(argc,argv,"add_client");
ros::NodeHandle nh;
ros::ServiceClient client =nh.serviceClient<my_first_pkg::AddTwoInts>("add_two_ints");

my_first_pkg::AddTwoInts srv;

int a,b;

std::cout << "첫번째 숫자" ;
std::cin >> a;

std::cout << "두번째 숫자" ;
std::cin >> b;

srv.request.a = a;
srv.request.b = b;

if(client.call(srv))
{
ROS_INFO("Result: %ld",srv.response.sum);
}
else{
ROS_ERROR("service call failed");
}
return 0;
}

