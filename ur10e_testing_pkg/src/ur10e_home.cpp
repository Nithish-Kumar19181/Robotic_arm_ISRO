#include"rclcpp/rclcpp.hpp"
#include<chrono>
#include<moveit/move_group_interface/move_group_interface.h>

using namespace std::chrono_literals ;
using namespace std::placeholders ;

class MoveHome : public rclcpp::Node
{
    public:
        MoveHome()
        :Node("move_home_node")
        {
            timer_ = this->create_wall_timer(
                2s,std::bind(&MoveHome::init,this)
            ) ;
        }
    private:

        void init()
        {
            timer_ ->cancel() ;

            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                this->shared_from_this(),
                "ur_manipulator"
            );

            move_group_->setPlanningTime(10.0);
            move_group_->setMaxVelocityScalingFactor(0.2);
            move_group_->setMaxAccelerationScalingFactor(0.2);

            moveHome() ;
            
        }
        void moveHome()
        {
            std::map<std::string, double> home_position ;
            home_position["shoulder_pan_joint"]  =  0.40960574676343686;
            home_position["shoulder_lift_joint"] = -1.6747645009139471;
            home_position["elbow_joint"]         = -2.401686248302344;
            home_position["wrist_1_joint"]       =  0.9661313426465519;
            home_position["wrist_2_joint"]       =  1.1646508483476774 ;
            home_position["wrist_3_joint"]       = -3.1578720449159454;
            

            move_group_->setJointValueTarget(home_position) ;
            moveit::planning_interface::MoveGroupInterface::Plan plan;

            auto result = move_group_->plan(plan) ;
            if(result != moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_ERROR(get_logger(),"PLANNING FAILED") ;
                return ;
            }
            RCLCPP_INFO(get_logger(),"PLAN SUCCESS MOVED TO HOME....") ;
            executePlan(plan, result) ;

        }

        void executePlan(auto plan, auto result)
        {
            if(result == moveit::core::MoveItErrorCode::SUCCESS)
            {
                auto exec_result = move_group_->execute(plan) ;
                if(exec_result != moveit::core::MoveItErrorCode::SUCCESS)
                {
                    RCLCPP_ERROR(get_logger(),"EXECUTION FAILED") ;
                    return ;
                }
                RCLCPP_INFO(get_logger(),"EXECUTION SUCCESS MOVED TO HOME") ;
            
            }
        }

        rclcpp::TimerBase::SharedPtr timer_ ;
        rclcpp::TimerBase::SharedPtr timer2_ ;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_ ;
        
};

int main(int argc , char **argv)
{
    rclcpp::init(argc,argv) ;
    rclcpp::spin(std::make_shared<MoveHome>()) ;
    rclcpp::shutdown() ;
    return 0 ;
}