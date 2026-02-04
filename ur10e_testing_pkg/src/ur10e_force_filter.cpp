#include"rclcpp/rclcpp.hpp"
#include"geometry_msgs/msg/wrench_stamped.hpp"
#include<chrono>
#include"std_msgs/msg/string.hpp"

using namespace std::chrono_literals ;
using namespace std::placeholders ;

class FilteredForce : public rclcpp::Node
{
    const float alpha = 0.05 ;
    bool init = false ;
    geometry_msgs::msg::WrenchStamped prev_ ;

    public:
        FilteredForce()
        :Node("filtered_force_node")
        {
            auto qos = rclcpp::QoS(rclcpp::KeepLast(10)) ;

            wrench_sub = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
                "/wrench",qos,std::bind(&FilteredForce::WrenchCallback,this,_1)
            ) ;

            filtered_wrench_pub = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
                "/cartesian_compliance_controller/ft_sensor_wrench",qos) ;
        }
    private:

        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr filtered_wrench_pub ;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub ;
        
        void WrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
        {   
            geometry_msgs::msg::WrenchStamped filtered_msg ;
            filtered_msg.header = msg->header ;
            if(!init)
            {
                filtered_msg.wrench = msg->wrench ;
                prev_.wrench = filtered_msg.wrench ;
                init = true ;
            }else
            {
                filtered_msg.wrench.force.x = alpha * (msg->wrench.force.x) + (1.0-alpha) * (prev_.wrench.force.x) ;
                filtered_msg.wrench.force.y = alpha * (msg->wrench.force.y) + (1.0-alpha) * (prev_.wrench.force.y) ;
                filtered_msg.wrench.force.z = alpha * (msg->wrench.force.z) + (1.0-alpha) * (prev_.wrench.force.z) ;

                filtered_msg.wrench.torque.x = alpha * (msg->wrench.torque.x) + (1.0-alpha) * (prev_.wrench.torque.x) ;
                filtered_msg.wrench.torque.y = alpha * (msg->wrench.torque.y) + (1.0-alpha) * (prev_.wrench.torque.y) ;
                filtered_msg.wrench.torque.z = alpha * (msg->wrench.torque.z) + (1.0-alpha) * (prev_.wrench.torque.z) ;
                
                prev_.wrench = filtered_msg.wrench ;

                filtered_wrench_pub->publish(filtered_msg) ;
            }
        }

} ;

int main(int argc , char **argv)
{
    rclcpp::init(argc,argv) ;
    rclcpp::spin(std::make_shared<FilteredForce>()) ;
    rclcpp::shutdown();
    return 0 ;

}