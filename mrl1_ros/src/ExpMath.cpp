#include "ExpMath.hpp"

namespace GroupMathSE{

    Eigen::Matrix3d ExpMath::SE2_ad(Eigen::Matrix3d X)
    {
        Eigen::Matrix2d M;
        M << 0, 1,
             -1,0;

        Eigen::Vector2d v(X(1-1,3-1), X(2-1,3-1));

        double alpha = X(2-1,1-1);

        Eigen::Matrix3d adX;

        adX << -alpha*M, M*v,
               Eigen::RowVectorXd::Zero(2), 0;


        return adX;
    }


}
