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

         void ExpMath::convolutionSE2(Eigen::Matrix3d mu1,Eigen::Matrix3d mu2,Eigen::Matrix3d cov1,Eigen::Matrix3d cov2,Eigen::Matrix3d& mu_bar,Eigen::Matrix3d& cov_bar)
         { 
                mu_bar = mu1*mu2;

                 double E_basis[3][3][3]=
                {
                    {
                        {0,-1,0},
                        {1,0,0},
                        {0,0,0},
                    },

                    {
                        {0,0,1},
                        {0,0,0},
                        {0,0,0},
                    },

                    {
                        {0,0,0},
                        {0,0,1},
                        {0,0,0},
                    },

                };


        Eigen::Matrix3d A,B,C,D,E,temp;
        temp = GroupMathSE::ExpMath::SE2_Adjoint(mu2.inverse());
        A =temp*cov1*temp.transpose();
        B = cov2;


        C = Eigen::MatrixXd::Zero(3,3);
        D = Eigen::MatrixXd::Zero(3,3);
        E = Eigen::MatrixXd::Zero(3,3);

        for(int i = 0; i < 3; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                Eigen::Matrix3d Ei,Ej;

                Ei = GroupMathSE::ExpMath::mapArray3D(E_basis,i);
                Ej = GroupMathSE::ExpMath::mapArray3D(E_basis,j);


                Eigen::Matrix3d adi,adj;
                adi = GroupMathSE::ExpMath::SE2_ad(Ei);
                adj = GroupMathSE::ExpMath::SE2_ad(Ej);

                C = C + adi*B*adj.transpose()*A(i,j);
                D = D + adi*adj*A(i,j)*B + (adi*adj*A(i,j)*B).transpose();
                E = E + adi*adj*B(i,j)*A + (adi*adj*B(i,j)*A).transpose();
            }
        }

        C = C/4;
        D = D/12;
        E = E/12;

        cov_bar = A + B + C + D + E;


         }

           
// integration using the 3/8 simpson rule
double ExpMath::SimpsonIntegrate(const ex& t,ex f,double a, double b)
{
    ex res;

    double h, c, d;
    h = (b-a)/8;
    c = (2*a+b)/3;
    d = (a+2*b)/3;

    res = evalf(h*(f.subs(t==a) + 3*f.subs(t==c) + 3*f.subs(t==d) + f.subs(t==b)));

   // res = evalf(f.subs(t==a));

    double out;

     if (is_a<numeric>(res)) 
     {
          out = ex_to<numeric>(res).to_double();
     }

    return out;
}



}

