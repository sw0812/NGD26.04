#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class PublisherNode : public rclcpp::Node //node 생성
{
public:
PublisherNode() : Node("publisher_node")
{
 publisher_ = this->create_publisher<std_msgs::msg::String>("chatter",10); //publisher 생성
 timer_ = this->create_wall_timer( //1초마다 실행
 1s,
 std::bind(&PublisherNode::timer_callback,this));
}

private:
 void timer_callback()
{
 auto msg = std_msgs::msg::String();

 msg.data="Hello ROS2!";

 publisher_->publish(msg); //메시지 보냄
 
 RCLCPP_INFO(this->get_logger(), "Publish:'%s'", msg.data.c_str());
}

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
rclcpp::init(argc,argv);
rclcpp::spin(std::make_shared<PublisherNode>()); //node안꺼지게
rclcpp::shutdown();

return 0;
}
