#include "ExpMath.hpp"
#include "eigen3/Eigen/LU"

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

    Eigen::Matrix3d ExpMath::SE2_Adjoint(Eigen::Matrix3d g)
    {
        Eigen::Matrix2d M;
        M << 0, 1,
             -1,0;
        
        Eigen::Vector2d t(g(1-1,3-1),g(2-1,3-1));
        Eigen::Matrix2d R;

        R = M.block(0,0,2,2);

        Eigen::Matrix3d Adg;

        Adg << R, M*t,
               Eigen::RowVectorXd::Zero(2),1;

        return Adg;

    }


      Eigen::Matrix3d ExpMath::ExpToSE2(Eigen::Vector3d ep)
      {
          double v1 = ep(1);
          double v2 = ep(2);
          double alpha = ep(3);

          Eigen::Matrix2d R;

          R << cos(alpha), -sin(alpha),
               sin(alpha), cos(alpha);

          Eigen::Matrix2d tempM;

          tempM << sin(alpha), -(1-cos(alpha)),
                   (1-cos(alpha)), sin(alpha);

          tempM = (1/alpha)*tempM;

          Eigen::Vector2d t(v1,v2);

          t = tempM*t;

          Eigen::Matrix3d g;

          g << R, t,
              0,0,0,1;

          return g;

      }


        Eigen::Vector3d ExpMath::SE2ToExp(Eigen::Matrix3d g)
        {
            double alpha = atan2(g(2,1),g(2,2));

            Eigen::Matrix2d tempM;

            tempM << sin(alpha), -(1-cos(alpha)),
                     (1-cos(alpha)), sin(alpha);

            tempM = (1/alpha)*tempM;

            Eigen::Vector2d t(g(1-1,3-1),g(2-1,3-1));
            
            Eigen::Vector2d v;

            v << tempM.inverse()*t;

            Eigen::Vector3d ep(v(1),v(2),alpha);

           return ep;  
        }


         Eigen::Vector3d ExpMath::SE2ToXYTheta(Eigen::Matrix3d g)
         {
              Eigen::Vector3d X(g(1-1,3-1),g(2-1,3-1), atan2(g(2-1,1-1),g(2-1,2-1)));

              return X;

         }


        Eigen::Matrix3d ExpMath::skew(Eigen::Vector3d x)
        {
            Eigen::Matrix3d X;

            X << 0, -x(3-1), x(2-1),
                 x(3-1), 0, -x(1-1),
                 -x(2-1), x(1-1), 0;

            return X;
        }

         Eigen::Vector3d ExpMath::vee(Eigen::Matrix3d X)
         {
             Eigen::Vector3d x(X(1-1,3-1),X(2-1,3-1),-X(1-1,2-1));

             return x;
         }

         Eigen::Matrix3d ExpMath::wedge(Eigen::Vector3d x)
         {
             Eigen::Matrix3d XX;

             XX << 0, -x(3-1), x(1-1),
                  x(3-1), 0, x(2-1),
                    0,    0,   0;

             return XX;
         }

         Eigen::Matrix3d ExpMath::XYThetaToSE2(Eigen::Vector3d cc)
         {
             Eigen::Matrix3d g;

             g << cos(cc(3-1)), -sin(cc(3-1)), cc(1-1),
                  sin(cc(3-1)), cos(cc(3-1)), cc(2-1),
                  0,0,1;

             return g;
         }

        Eigen::Matrix3d ExpMath::mapArray3D(double a[][3][3], int n)
        {
            Eigen::Matrix3d M;

            M << a[n][0][0],a[n][0][1],a[n][0][2],
                 a[n][1][0],a[n][1][1],a[n][1][2],
                 a[n][2][0],a[n][2][1],a[n][2][2];

            return M;
        }
}



























