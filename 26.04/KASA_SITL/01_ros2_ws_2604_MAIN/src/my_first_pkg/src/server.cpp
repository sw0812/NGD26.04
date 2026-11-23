#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "my_first_pkg/srv/add_two_ints.hpp"

using AddTwoInts = my_first_pkg::srv::AddTwoInts;

class ServerNode: public rclcpp::Node
{
public:
ServerNode() : Node("server_node")
{
 service_=this->create_service<AddTwoInts>(
 "add_two_ints",
 std::bind(
  &ServerNode::add,
  this,
  std::placeholders::_1,
  std::placeholders::_2));
}

private:
 void add(
  const std::shared_ptr<AddTwoInts::Request> request,
  std::shared_ptr<AddTwoInts::Response> response)
{
  response->sum = request ->a + request ->b;

  RCLCPP_INFO(
   this->get_logger(),
   "%ld + %ld = %ld",
    request->a,
    request->b,
    response->sum);
}

rclcpp::Service<AddTwoInts>::SharedPtr service_;
};

int main (int argc,char * argv[])
{
 rclcpp::init(argc,argv);
 rclcpp::spin(std::make_shared<ServerNode>());
 rclcpp::shutdown();

 return 0;
}
