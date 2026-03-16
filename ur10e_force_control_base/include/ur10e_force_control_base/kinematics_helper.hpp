#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include"ur10e_force_control_base/robot_state.hpp"
#include <memory>
#include <string>
#include <array>
#include <algorithm>

namespace kinematics_helper
{

  class KinematicsHelper
  {
    public:
      KinematicsHelper() = default;

      /**
     * @brief Initilises kdl and fk solver.
     * @param urdf_string Has the robot description from the topic /robot_description.
     * @param base_link .
     * @param tool0 .
     */
      bool init(const std::string & urdf_string, const std::string & base_link = "base_link",
        const std::string & tool_link  = "tool0");

      /**
     * @brief Computes the fk for the current RobotState.
     * @param robot_state Has the current robot_state.
     */
      bool computeFK(RobotState &robot_state) const;
      /**
     * @brief Gets the computed positions.
     * @param frame Has the robot's Frame.
     * @param x,y,z Are the fk slutions.
     */
      static void getPosition(const KDL::Frame &frame, double & x, double &y, double & z);

      static void getQuaternion(const KDL::Frame &frame, double &qx, double &qy, double &qz, double &qw);

      bool isReady() const { return ready_; }

      unsigned int numJoints() const;

  private:
    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    unsigned int num_joints_{0};
    bool ready_{false};
  };

}  // namespace kinematics_helper
