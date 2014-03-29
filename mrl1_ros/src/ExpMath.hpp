#ifndef EXPMATH_HPP_
#define EXPMATH_HPP_

#include <eigen3/Eigen/Dense>
#include <sstream>
#include <vector>
#include <iostream>
#include <math.h>

namespace GroupMathSE{

    class ExpMath
    {
       public:

        // calculate small adjoint for SE2
        // input: Lie algebra se(2)
        static Eigen::Matrix3d SE2_ad(Eigen::Matrix3d X);

        // calculate Adjoint for SE2
        // input: homogeneous matrix SE2
        static Eigen::Matrix3d SE2_Adjoint(Eigen::Matrix3d g);

        // convert exponential coordinate to homoegeneous 
        // input: vector3d exponential coordinate in [v1,v2,alpha]
        static Eigen::Matrix3d ExpToSE2(Eigen::Vector2d ep);

        // convert homoegeneous matrix in SE2 to exponential coordinates
        // input: 3x3 homoegeneous matrix in SE2
        static Eigen::Vector3d SE2ToExp(Eigen::Matrix3d g);
    };  

        // convert 3x3 homoegeneous matrix to a vector of x y theta
        static Eigen::Vector3d SE2ToXYTheta(Eigen::Matrix3d g);

}


#endif
