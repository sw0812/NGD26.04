#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "my_first_pkg/action/countdown.hpp"

using namespace std::chrono_literals;

class ActionServer : public rclcpp::Node
{
public:
  using Countdown = my_first_pkg::action::Countdown;
  using GoalHandleCountdown =
   rclcpp_action::ServerGoalHandle<Countdown>;

 ActionServer()
  : Node("action_server")
{
  action_server_ = rclcpp_action::create_server<Countdown>(
   this,
   "countdown",
        std::bind(
            &ActionServer::handle_goal,
            this,
            std::placeholders::_1,
            std::placeholders::_2),
        std::bind(
            &ActionServer::handle_cancel,
            this,
            std::placeholders::_1),
        std::bind(
            &ActionServer::handle_accepted,
            this,
            std::placeholders::_1));
}

private:
 rclcpp_action::Server<Countdown>::SharedPtr action_server_;

 rclcpp_action::GoalResponse handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const Countdown::Goal> goal)
{
  RCLCPP_INFO(
  this->get_logger(),
  "Goal received : %d",
  goal->target);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

 rclcpp_action::CancelResponse handle_cancel(
  const std::shared_ptr<GoalHandleCountdown>)
{

 RCLCPP_INFO(this->get_logger(),"Goal canceled");
 return rclcpp_action::CancelResponse::ACCEPT;

}

void handle_accepted(
  const std::shared_ptr<GoalHandleCountdown> goal_handle)
{
  std::thread(
   std::bind(
    &ActionServer::execute,
    this,
    goal_handle))
     .detach();
}

void execute(
 const std::shared_ptr<GoalHandleCountdown> goal_handle)
{
 auto feedback = std::make_shared<Countdown::Feedback>();
 auto result = std::make_shared<Countdown::Result>();

int target = goal_handle->get_goal()->target;

for (int i = 0; i <= target; i++)
{
 feedback->current = i;

 goal_handle->publish_feedback(feedback);

 RCLCPP_INFO(
  this->get_logger(),
  "Current : %d",
   i);

 std::this_thread::sleep_for(1s);
}

  result->success = true;

        goal_handle->succeed(result);

        RCLCPP_INFO(
            this->get_logger(),
            "Finished");
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ActionServer>());

    rclcpp::shutdown();

    return 0;
}
