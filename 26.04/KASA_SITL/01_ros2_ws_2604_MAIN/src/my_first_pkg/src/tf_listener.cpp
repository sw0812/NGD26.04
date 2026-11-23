#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class TFListener : public rclcpp::Node
{
public:
    TFListener()
    : Node("tf_listener"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
    {
      timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&TFListener::timer_callback, this)
      );
    }

private:
    void timer_callback();

    rclcpp::TimerBase::SharedPtr timer_;
   
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

    void TFListener::timer_callback()
    {
      try
      {
         auto transform = tf_buffer_.lookupTransform(
             "world",
             "robot",
             tf2::TimePointZero
         );
         
      RCLCPP_INFO(
         this->get_logger(),
         "x = %.2f, y = %.2f z =  %.2f",
         transform.transform.translation.x,
         transform.transform.translation.y,
         transform.transform.translation.z
      );
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(),"%s", ex.what());
}
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TFListener>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    
    return 0;
}
