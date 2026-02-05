#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

using std::placeholders::_1;

class FilteredForce : public rclcpp::Node
{
    const float alpha = 0.05;
    bool init = false;

    geometry_msgs::msg::Wrench prev_;

public:
    FilteredForce() : Node("filtered_force_node")
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

        // Subscribe plain Wrench
        wrench_sub_ = this->create_subscription<geometry_msgs::msg::Wrench>(
            "/wrench", qos,
            std::bind(&FilteredForce::wrenchCallback, this, _1));

        // Publish WrenchStamped
        filtered_pub_ =
            this->create_publisher<geometry_msgs::msg::WrenchStamped>(
                "/cartesian_compliance_controller/ft_sensor_wrench", qos);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr wrench_sub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr filtered_pub_;

    void wrenchCallback(const geometry_msgs::msg::Wrench::SharedPtr msg)
    {
        geometry_msgs::msg::WrenchStamped filtered_msg;

        // Add timestamp + frame
        filtered_msg.header.stamp = this->now();
        filtered_msg.header.frame_id = "tool0";  // change if needed

        if (!init)
        {
            filtered_msg.wrench = *msg;
            prev_ = *msg;
            init = true;
        }
        else
        {
            filtered_msg.wrench.force.x =
                alpha * msg->force.x + (1.0 - alpha) * prev_.force.x;
            filtered_msg.wrench.force.y =
                alpha * msg->force.y + (1.0 - alpha) * prev_.force.y;
            filtered_msg.wrench.force.z =
                alpha * msg->force.z + (1.0 - alpha) * prev_.force.z;

            filtered_msg.wrench.torque.x =
                alpha * msg->torque.x + (1.0 - alpha) * prev_.torque.x;
            filtered_msg.wrench.torque.y =
                alpha * msg->torque.y + (1.0 - alpha) * prev_.torque.y;
            filtered_msg.wrench.torque.z =
                alpha * msg->torque.z + (1.0 - alpha) * prev_.torque.z;

            prev_ = filtered_msg.wrench;
        }

        filtered_pub_->publish(filtered_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FilteredForce>());
    rclcpp::shutdown();
    return 0;
}
