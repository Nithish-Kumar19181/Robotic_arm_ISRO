#include"ur10e_force_control_base/mapping/trajectory_math.hpp"

namespace trajectory_math
{
    double Scurve(double t)
    {
        t = std::clamp(t, 0.0, 1.0);
        return (10*t*t*t) - (15*t*t*t*t) + (6*t*t*t*t*t);
    }
}