#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "my_first_pkg/srv/add_two_ints.hpp"

using namespace std::chrono_literals;
using AddTwoInts = my_first_pkg::srv::AddTwoInts;

class ClientNode : public rclcpp::Node
{
public:
 ClientNode() : Node("client_node")
{
 client_= this->create_client<AddTwoInts>("add_two_ints");

 while (!client_->wait_for_service(1s))
{
  RCLCPP_INFO(this->get_logger(),"waiting for service...");
}

auto request = std::make_shared<AddTwoInts::Request>();

request->a = 3;
request->b = 5;

auto future = client_->async_send_request(request); 

auto result = future.get();

RCLCPP_INFO(
  this->get_logger(),
  "Result : %ld",
  result->sum);
}

private:
 rclcpp::Client<AddTwoInts>::SharedPtr client_;
};

int main(int argc, char * argv[])
{

 rclcpp::init(argc, argv);
 rclcpp::spin(std::make_shared<ClientNode>());
 rclcpp::shutdown();

 return 0;
}
