#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class StaticTFBroadcaster : public rclcpp::Node
{

public:
 StaticTFBroadcaster()
 : Node("static_tf_broadcaster")
 {
  static_broadcaster_ =
   std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

 geometry_msgs::msg::TransformStamped transform;
 
 transform.header.stamp = this->get_clock()->now();

 transform.header.frame_id =  "robot";
 transform.child_frame_id = "camera";

 transform.transform.translation.x= 0.2;
 transform.transform.translation.y= 0.0;
 transform.transform.translation.z= 0.5;

 transform.transform.rotation.x = 0.0;
 transform.transform.rotation.y = 0.0;
 transform.transform.rotation.z = 0.0;
 transform.transform.rotation.w = 1.0;

 static_broadcaster_->sendTransform(transform);

 RCLCPP_INFO(this->get_logger(),"Static TF Published");
}

private:
     std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<StaticTFBroadcaster>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
