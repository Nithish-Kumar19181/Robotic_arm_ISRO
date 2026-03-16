#include"ur10e_force_control_base/wrench_filter.hpp"

namespace wrench_filter
{
    WrenchFilter::WrenchFilter(double alpha)
    :alpha_(alpha),initialized_(false) {}

    geometry_msgs::msg::WrenchStamped WrenchFilter::filter(const geometry_msgs::msg::WrenchStamped &input)
    {
        if(!initialized_)
        {
            prev_wrench_ = input ;
            initialized_ = true ;
            return input ;
        }
        
    geometry_msgs::msg::WrenchStamped filtered ;

    // Forces
    filtered.wrench.force.x =
        alpha_ * input.wrench.force.x +
        (1.0 - alpha_) * prev_wrench_.wrench.force.x;

    filtered.wrench.force.y =
        alpha_ * input.wrench.force.y +
        (1.0 - alpha_) * prev_wrench_.wrench.force.y;

    filtered.wrench.force.z =
        alpha_ * input.wrench.force.z +
        (1.0 - alpha_) * prev_wrench_.wrench.force.z;

    // Torques
    filtered.wrench.torque.x =
        alpha_ * input.wrench.torque.x +
        (1.0 - alpha_) * prev_wrench_.wrench.torque.x;

    filtered.wrench.torque.y =
        alpha_ * input.wrench.torque.y +
        (1.0 - alpha_) * prev_wrench_.wrench.torque.y;

    filtered.wrench.torque.z =
        alpha_ * input.wrench.torque.z +
        (1.0 - alpha_) * prev_wrench_.wrench.torque.z;

    prev_wrench_ = filtered;

    return filtered;
    }

    void WrenchFilter::reset()
    {
        initialized_ = false ;
    }
}

