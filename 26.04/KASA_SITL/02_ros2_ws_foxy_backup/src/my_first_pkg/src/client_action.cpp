#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "my_first_pkg/action/countdown.hpp"

using Countdown = my_first_pkg::action::Countdown;

class ActionClient : public rclcpp::Node
{
public:
  using GoalHandleCountdown =
   rclcpp_action::ClientGoalHandle<Countdown>;

  ActionClient()
     : Node("action_client")
{
 action_client_=
  rclcpp_action::create_client<Countdown>(
   this,
   "countdown");

 send_goal();
}

private:
void send_goal()
{
 while (!action_client_->wait_for_action_server(std::chrono::seconds(1)))
 {
    RCLCPP_INFO(this->get_logger(), "Wating for action server...");
  }

 auto goal_msg= Countdown::Goal();
 goal_msg.target = 5;

 auto send_goal_options =
    rclcpp_action::Client<Countdown>::SendGoalOptions();

 action_client_->async_send_goal(
    goal_msg,
    send_goal_options);
  }

  rclcpp_action::Client<Countdown>::SharedPtr action_client_;
};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ActionClient>());

  rclcpp::shutdown();

  return 0;
}
