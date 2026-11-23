#include <memory>
#include <functional>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class TFBroadcaster : public rclcpp::Node
{
 public:
  TFBroadcaster ()
  : Node("tf_broadcaster")
  {
 tf_broadcaster_=
  std::make_shared<tf2_ros::TransformBroadcaster>(this);

 timer_ = this->create_wall_timer(
   std::chrono::seconds(1),
   std::bind(&TFBroadcaster::broadcast_tf,this)
);
 }

 private:
  void broadcast_tf()
  {
    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = "world";
    transform.child_frame_id = "robot";

    transform.transform.translation.x = 1.0;
    transform.transform.translation.y = 2.0;
    transform.transform.translation.z = 0.0;

    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(transform);

    RCLCPP_INFO(this->get_logger(), "TF Sent!");
}
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
 rclcpp::init(argc, argv);
 
 auto node = std::make_shared<TFBroadcaster>();
    rclcpp::spin(node);

 rclcpp::shutdown();

 return 0;

}
