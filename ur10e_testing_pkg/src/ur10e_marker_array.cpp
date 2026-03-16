#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

using namespace std::chrono_literals;

class ToolMarkerPublisher : public rclcpp::Node
{
public:
  ToolMarkerPublisher()
  : Node("tool_marker_trail_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/tool_marker_array", 10);

    timer_ = create_wall_timer(
      100ms, std::bind(&ToolMarkerPublisher::publish_marker, this));

    RCLCPP_INFO(get_logger(), "Tool marker trail publisher started");
  }

private:
  void publish_marker()
  {
    geometry_msgs::msg::TransformStamped tf;

    try
    {
      tf = tf_buffer_.lookupTransform(
        "base_link", "tool0", tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TF not available: %s", ex.what());
      return;
    }

    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = "base_link";
    sphere.header.stamp = now();
    sphere.ns = "tool_trail";
    sphere.id = marker_id_++;  
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;

    sphere.pose.position.x = tf.transform.translation.x;
    sphere.pose.position.y = tf.transform.translation.y;
    sphere.pose.position.z = tf.transform.translation.z;
    sphere.pose.orientation.w = 1.0;

    sphere.scale.x = 0.02;
    sphere.scale.y = 0.02;
    sphere.scale.z = 0.02;

    sphere.color.r = 1.0;
    sphere.color.g = 0.0;
    sphere.color.b = 0.0;
    sphere.color.a = 1.0;

    sphere.lifetime = rclcpp::Duration(0, 0);

    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(sphere);

    marker_pub_->publish(array);
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  int marker_id_ = 0;  
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ToolMarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}
