#pragma once

#include <geometry_msgs/msg/wrench_stamped.hpp>

namespace wrench_filter
{

class WrenchFilter
{
public:
    explicit WrenchFilter(double alpha = 0.05);

    geometry_msgs::msg::WrenchStamped
    filter(const geometry_msgs::msg::WrenchStamped &input);

    void reset();

private:
    double alpha_{0.05};
    bool initialized_{false};
    geometry_msgs::msg::WrenchStamped prev_wrench_;
};

}  // namespace wrench_filter