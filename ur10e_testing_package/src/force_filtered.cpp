#include"rclcpp/rclcpp.hpp"
#include"geometry_msgs/msg/wrench_stamped.hpp"
#include"std_msgs/msg/string.hpp"

using namespace std::chrono_literals ;
using namespace std::placeholders    ;

class ForceFilter
    : public rclcpp::Node
{
    //CONSTRUCTOR
    const float alpha_ = 0.05 ;
    bool init_ = false ;
    geometry_msgs::msg::Wrench prev_;
    
    public:
        ForceFilter()
            :Node("force_filter_node")
            {
                auto qos = rclcpp::SensorDataQoS().keep_last(1);

                wrench_sub = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
                    "/force_torque_sensor/wrench",qos,
                    std::bind(&ForceFilter::FilterCallback,this,_1)
                ) ;
                filtered_wrench_pub = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
                    "/filtered_wrench",qos
                ); 

                RCLCPP_INFO(this->get_logger(),"publishing filtered wrench stamped")  ;
            }
    private:

    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub ;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr filtered_wrench_pub ;

    void FilterCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        geometry_msgs::msg::WrenchStamped filtered_msg ;
        filtered_msg.header = msg->header ;
        
        if(!init_)
        {
            filtered_msg.wrench = msg->wrench ;
            prev_ = filtered_msg.wrench ;
            init_ = true ;
        }
        else
        {
            // x_filtered = a*x_0 + (1-a)*x_-1
            filtered_msg.wrench.force.x  = alpha_ * msg->wrench.force.x  + (1.0 - alpha_) * prev_.force.x;
            filtered_msg.wrench.force.y  = alpha_ * msg->wrench.force.y  + (1.0 - alpha_) * prev_.force.y;
            filtered_msg.wrench.force.z  = alpha_ * msg->wrench.force.z  + (1.0 - alpha_) * prev_.force.z;

            filtered_msg.wrench.torque.x = alpha_ * msg->wrench.torque.x + (1.0 - alpha_) * prev_.torque.x;
            filtered_msg.wrench.torque.y = alpha_ * msg->wrench.torque.y + (1.0 - alpha_) * prev_.torque.y;
            filtered_msg.wrench.torque.z = alpha_ * msg->wrench.torque.z + (1.0 - alpha_) * prev_.torque.z;

            prev_ = filtered_msg.wrench;

            filtered_wrench_pub->publish(filtered_msg) ;
        }
    }

};

int main(int argc , char **argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ForceFilter>());
    rclcpp::shutdown() ;
    return 0 ;
}