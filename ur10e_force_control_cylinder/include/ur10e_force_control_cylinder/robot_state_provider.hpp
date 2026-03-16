#pragma once

#include <string>

#include "ur10e_force_control_base/robot_state.hpp"
#include "ur10e_force_control_base/kinematics_helper.hpp"

namespace robot_state_provider
{

class RobotStateProvider
{
public:

    RobotStateProvider() = default;

    bool initialize(const std::string& urdf,
                    const std::string& base,
                    const std::string& tool);

    bool updateRobotState(RobotState& state);

private:

    kinematics_helper::KinematicsHelper kinematics_;
};

}

