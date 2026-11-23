#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ParameterNode : public rclcpp::Node
{

public:
 ParameterNode() : Node("parameter_node")
{
 this->declare_parameter("name","ROS2");

 timer_ = this->create_wall_timer(
  1s,
  std::bind(&ParameterNode::timer_callback,this));
}


private:
 void timer_callback()
 {
  std::string name =
   this->get_parameter("name").as_string();

  RCLCPP_INFO(
   this->get_logger(),
   "Hello %s",
   name.c_str());
   }

 rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc,char * argv[])
{

rclcpp::init(argc,argv);
rclcpp::spin(std::make_shared<ParameterNode>());
rclcpp::shutdown();

return 0;
}
