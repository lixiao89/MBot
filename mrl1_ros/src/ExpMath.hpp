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
        static Eigen::Matrix3d SE2_ad(Eigen::Matrix3d X);

    };

}


#endif
