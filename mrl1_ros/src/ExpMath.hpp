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
        static Eigen::Matrix3d ExpToSE2(Eigen::Vector3d ep);

        // convert homoegeneous matrix in SE2 to exponential coordinates
        // input: 3x3 homoegeneous matrix in SE2
        static Eigen::Vector3d SE2ToExp(Eigen::Matrix3d g);
     

        // convert 3x3 homoegeneous matrix to a vector of x y theta
        static Eigen::Vector3d SE2ToXYTheta(Eigen::Matrix3d g);

        // calculate the skew symmetric matrix of vector x
        static Eigen::Matrix3d skew(Eigen::Vector3d x);
        // calculate the SE2 vee
        static Eigen::Vector3d vee(Eigen::Matrix3d X);

        // calculate the SE2 wedge
        static Eigen::Matrix3d wedge(Eigen::Vector3d x);

        // convert x y theta to SE2 homoegeneous matrix
        static Eigen::Matrix3d XYThetaToSE2(Eigen::Vector3d cc);

        // map the n_th slice of "a" to a 3x3 matrix
        static Eigen::Matrix3d mapArray3D(double a[][3][3], int n);

        static Eigen::Matrix3d polarToSE2(double s);

        // calculate the convolution of two SE2 Gaussians
        static void convolutionSE2(Eigen::Matrix3d mu1,Eigen::Matrix3d mu2,Eigen::Matrix3d cov1,Eigen::Matrix3d cov2,Eigen::Matrix3d& mu_bar,Eigen::Matrix3d& cov_bar);
 
 
       
 
    };
}


#endif
