#include"ur10e_force_control_base/kinematics_helper.hpp"

namespace kinematics_helper
{
    bool KinematicsHelper::init(const std::string &urdf_string,
        const std::string &base_link, const std::string &tool_link)
    {
        urdf::Model model ;
        model.initString(urdf_string) ;
        KDL::Tree tree ;
        kdl_parser::treeFromUrdfModel(model,tree) ;
        tree.getChain(base_link,tool_link,chain_) ;
        num_joints_ = chain_.getNrOfJoints() ;

        fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_) ;
        if(fk_solver_)
        {
            ready_ = true ;
            return ready_;
        }
        ready_ = false ;
        return ready_ ;
    }

    bool KinematicsHelper::computeFK(RobotState &robot_state) const 
    {
        robot_state.frame_ = KDL::Frame::Identity() ;
        if(!ready_ || robot_state.joint_state.position.size()<num_joints_)
        {
            return false ;
        }
        KDL::JntArray joints(num_joints_) ;
        for(size_t i=0; i<num_joints_; ++i)
        {
            joints(i) = robot_state.joint_state.position[i] ;
        }
        // change this for driver for actual arm 
        int status = fk_solver_->JntToCart(joints, robot_state.frame_) ;
        return status >=0 ; 
    }

    void KinematicsHelper::getPosition(const KDL::Frame &frame, double &x, double &y, double &z)
    {
        x = frame.p.x() ;
        y = frame.p.y() ;
        z = frame.p.z() ;
        return  ;
    }

    void KinematicsHelper::getQuaternion(const KDL::Frame &frame, double &qx, double &qy, double &qz, double &qw)
    {
        frame.M.GetQuaternion(qx,qy,qz,qw) ;
        return ;
    }

    unsigned int KinematicsHelper::numJoints() const 
    {
        return num_joints_ ;
    }
}

