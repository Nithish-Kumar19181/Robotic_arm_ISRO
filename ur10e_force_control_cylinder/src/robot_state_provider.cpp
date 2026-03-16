#include "ur10e_force_control_cylinder/robot_state_provider.hpp"

namespace robot_state_provider
{

    bool RobotStateProvider::initialize(const std::string& urdf,const std::string& base,
        const std::string& tool)
    {
        if(kinematics_.isReady())
            return true;

        return kinematics_.init(urdf, base, tool);
    }

    bool RobotStateProvider::updateRobotState(RobotState& state)
    {
        if(!kinematics_.isReady())
            return false;

        if(!kinematics_.computeFK(state))
            return false;

        return true;
    }

}