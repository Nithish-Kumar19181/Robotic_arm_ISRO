#include "ur10e_force_control_base/force_controller.hpp"

namespace force_control
{

    ForceController::ForceController(double kp, double ki, double integral_limit, double max_output_rate)
        : kp_(kp),
        ki_(ki),
        integral_limit_(integral_limit),
        max_output_rate_(max_output_rate),
        integral_(0.0){}

    void ForceController::computeAdmittance(double measured_force, double desired_force, double dt, double &output)
    {
        if (dt <= 0.0)
            return ;

        double error = measured_force - desired_force;

        integral_ += error * dt;
        integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);

        output = kp_ * error + ki_ * integral_;

        output = std::clamp(output, -max_output_rate_, max_output_rate_);
        return  ;
    }

    void ForceController::resetForceControl()
    {
        integral_ = 0.0;
    }

} 