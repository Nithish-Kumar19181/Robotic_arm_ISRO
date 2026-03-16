#pragma once

#include <algorithm>
#include <utility>

namespace force_control {

class ForceController {
public:
  ForceController() = default;

  ForceController(double kp, double ki, double integral_limit,
                  double max_output_rate);

  void computeAdmittance(double measured_force, double desired_force, double dt,
                         double &output);

  void resetForceControl();

private:
  double kp_{0.5};
  double ki_{0.0001};
  double integral_limit_{2.0};
  double max_output_rate_{0.01};

  double integral_{0.0};
};

} // namespace force_control