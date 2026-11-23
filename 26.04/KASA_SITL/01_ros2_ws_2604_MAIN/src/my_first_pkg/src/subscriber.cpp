#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SubscriberNode : public rclcpp::Node //SubscriberNode란 class생성,rclcpp::Node상속
{



public:
 SubscriberNode() : Node("subscriber_node") //객체생성
 {
  subscription_ =
  this->create_subscription<std_msgs::msg::String>(//상속기능 this는 객체
   "chatter",
   10,
   std::bind(&SubscriberNode::topic_callback, this, std::placeholders::_1));//메시지 입력시 콜백 실행
}



private:
 void topic_callback(const std_msgs::msg::String::SharedPtr msg) //call back은 멤버함수
 {
  RCLCPP_INFO(this->get_logger(),"I heard : '%s'",msg->data.c_str());
}

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};



int main (int argc,char * argv[])
{
 rclcpp::init(argc,argv);
 rclcpp::spin(std::make_shared<SubscriberNode>());
 rclcpp::shutdown();




 return 0;
}
