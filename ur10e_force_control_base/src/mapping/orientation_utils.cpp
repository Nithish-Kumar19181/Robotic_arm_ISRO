#include "ur10e_force_control_base/mapping/orientation_utils.hpp"

namespace orientation_utils
{
    void computeRadialOrientation(double theta, double &qx, double &qy, double &qz, double &qw)
    {
        KDL::Vector z_axis(std::cos(theta),std::sin(theta),0.0) ;
        z_axis.Normalize() ;
        KDL::Vector up(0.0,0.0,1) ;
        KDL::Vector x_axis = up*z_axis ;
        if(x_axis.Norm() < 1e-6)
        {
            x_axis = KDL::Vector(1,0,0) ;
        }
        x_axis.Normalize() ;

        KDL::Vector y_axis = z_axis * x_axis ;
        y_axis.Normalize() ;

        KDL::Rotation R(x_axis,y_axis,z_axis) ;
        R.GetQuaternion(qx, qy, qz, qw) ;
    }
            
    /**
    * \brief Computes Tool rotataion 
    * \return if everything went well 
    */

    void computeRotatedOrientation()
    {
        return ;
    }
}
