#include<rclcpp/rclcpp.hpp>
#include<geometry_msgs/msg/wrench_stamped.hpp>
#include<memory>
#include<algorithm>

#include "ur10e_force_control_base/wrench_filter.hpp"

using namespace std::placeholders ;
using namespace wrench_filter ;

class WrenchFilterNode : public rclcpp::Node
{   
    public:
        WrenchFilterNode()
        :Node("wrench_filter_node")
        {
            wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
                "/wrench",
                rclcpp::SensorDataQoS(),
                std::bind(&WrenchFilterNode::wrenchCallback,this,_1)

            ) ;

            filtered_wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
                "/cartesian_compliance_controller/ft_sensor_wrench",
                rclcpp::SensorDataQoS()
            ) ;

            filter_ = std::make_unique<WrenchFilter>(0.1) ;
        }

    private:
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_ ;
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr filtered_wrench_pub_ ;
        std::unique_ptr<WrenchFilter> filter_ ;

        void wrenchCallback(geometry_msgs::msg::WrenchStamped::SharedPtr msg)
        {
            auto filtered_wrench = filter_->filter(*msg) ;

            filtered_wrench_pub_->publish(filtered_wrench) ;
        }
} ;

int main(int argc , char** argv)
{
    rclcpp::init(argc,argv) ;
    rclcpp::spin(std::make_shared<WrenchFilterNode>()) ;
    rclcpp::shutdown() ;
    return 0 ;
}