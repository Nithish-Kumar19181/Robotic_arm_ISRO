# pragma once 

#include<kdl_parser/kdl_parser.hpp>

namespace orientation_utils
{
    void computeRadialOrientation(double theta, double &qx, double &qy, double &qz, double &qw) ;

    void computeRotatedOrientation() ; // write this later for tool rotation if needed 
}