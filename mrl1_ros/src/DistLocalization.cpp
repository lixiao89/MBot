#include "DistLocalization.hpp"


void DistLocalization::moveToLongestScan()
{
    
    int minRangeIndex = 0;
    float minRangeTemp = maxRange; 

    for(int i=0; i < rangeReadings.size();i++)
    {
        if(rangeReadings.at(i) < minRangeTemp)
        {
            minRangeTemp = rangeReadings.at(i);
            minRangeIndex = i;
        }
    }           

    if(angleReadings.at(minRangeIndex) > 0)
    {
        leftEffort.data = 2;
        rightEffort.data = -2;
    }
    else if(angleReadings.at(minRangeIndex) < 0)
    {
        leftEffort.data = -2;
        rightEffort.data = 2;

    }
    else
    {
        leftEffort.data = 3;
        rightEffort.data = 3;
    }

}

void DistLocalization::moveSinasoidal()
{   
    ros::Time time = ros::Time::now();

    float t;
    t = time.toSec();

    leftEffort.data = sin(t)+1;
    rightEffort.data = sin(t)+0.5;
}

//////////////////// Distributed EKF ///////////////////////
//






///////////////// Exponential Localization /////////////////

 // Main method to call
     void DistLocalization::expLocalization()
    {
         //fusion_with_sensor_noise(a_i, mu_i, cov_i, a_j, mu_j, cov_j, mu_m, cov_m, mu_i_bar, Sigma_i_bar);

        posEst.x = 1;
        posEst.y = 1;
        posEst.z = 0;

        poseEstimate_.publish(posEst);

    }

	    // returns xi and Si
void DistLocalization::generate_Si(Eigen::Matrix3d Sigma_i, Eigen::Vector3d& xi, Eigen::Matrix3d& Si)
{
    xi = GroupMathSE::ExpMath::SE2ToExp(Eigen::MatrixXd::Identity(3,3));
    Eigen::Matrix3d Xi;
    Xi = GroupMathSE::ExpMath::wedge(xi);

    Eigen::Matrix3d gamma_i;
    gamma_i = (Eigen::MatrixXd::Identity(3,3) + 0.5*GroupMathSE::ExpMath::SE2_ad(Xi));

    Si = gamma_i.transpose()*Sigma_i.inverse()*gamma_i;
}   


       
        // returns xm and Sm
void DistLocalization::generate_Sm(Eigen::Matrix3d m_im,Eigen::Matrix3d mu_m,Eigen::Matrix3d a_m,Eigen::Matrix3d a_i,Eigen::Matrix3d mu_i,Eigen::Matrix3d Sigma_m,Eigen::Vector3d& xm,Eigen::Matrix3d& Sm)
{
    Eigen::Matrix3d qm;
    qm = m_im*mu_m.inverse()*a_m.inverse()*a_i*mu_i;

    xm = GroupMathSE::ExpMath::SE2ToExp(qm);

    Eigen::Matrix3d Xm;
    Xm = GroupMathSE::ExpMath::wedge(xm);

    Eigen::Matrix3d gamma_m;
    gamma_m = (Eigen::MatrixXd::Identity(3,3)+0.5*GroupMathSE::ExpMath::SE2_ad(Xm));

    Eigen::Matrix3d temp;
    temp = GroupMathSE::ExpMath::SE2_Adjoint(m_im);
    temp = temp.inverse();

    Sm = gamma_m.transpose()*temp.transpose()*Sigma_m.inverse()*temp*gamma_m;

}


void DistLocalization::Fusion(Eigen::Matrix3d a_i,Eigen::Matrix3d mu_i,Eigen::Matrix3d cov_i,Eigen::Matrix3d A_m,Eigen::Matrix3d MU_m,Eigen::Matrix3d COV_m,Eigen::Matrix3d Mim, Eigen::Matrix3d& mu_i_bar, Eigen::Matrix3d& Sigma_i_bar)
{
    Eigen::Vector3d xi(0,0,0);
    Eigen::Matrix3d Si;
    Si = cov_i.transpose();

    Eigen::Vector3d xm;
    Eigen::Matrix3d Sm;

    generate_Sm(Mim,MU_m,A_m,a_i,mu_i,COV_m,xm,Sm);

    Eigen::Vector3d xbar;
    xbar = Sm.inverse()*(Si*xi+Sm*xm);
    Eigen::Matrix3d Xbar;
    Xbar = GroupMathSE::ExpMath::wedge(xbar);

    Eigen::Matrix3d gamma_bar;

    gamma_bar = (Eigen::MatrixXd::Identity(3,3)+0.5*GroupMathSE::ExpMath::SE2_ad(Xbar));

    Sigma_i_bar = gamma_bar*Sm.inverse()*gamma_bar.transpose();
    mu_i_bar = mu_i*(-Xbar.exp());
}


        // returns the final estimate update for mean (mu_i_bar) and covariance (Sigma_i_bar)
         void DistLocalization::fusion_with_sensor_noise(Eigen::Matrix3d a_i,Eigen::Matrix3d mu_i,Eigen::Matrix3d cov_i,Eigen::Matrix3d a_j,Eigen::Matrix3d mu_j,Eigen::Matrix3d cov_j,Eigen::Matrix3d mu_m,Eigen::Matrix3d cov_m,Eigen::Matrix3d& mu_i_bar,Eigen::Matrix3d& Sigma_i_bar)
{
    /* mu_i and cov_i are 3x3 matrices representing the estimated mean and covariance of the robot to be updated

         a_i is a 3x3 matrix denoting the initial position of robot i in world frame

        mu_i is a 3x3 matrix representing the mean of robot i relative to a_i

         a_j, mu_j, cov_j are the 3x3 matrices of robot j (robot to take measurement from)

         mu_m and cov_m are 3x3 the mean and covariance of the measurement  distribution (sensor model and gaussian in this case)*/

    // --------------- Perform the Convolution-like Calculation ------------------

    // Basis elements for SE(2)
    Eigen::Matrix3d E1,E2,E3;


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
   /* E1 << 0,-1,0,
          1,0,0,
          0,0,0;

    E2 << 0,0,1,
          0,0,0,
          0,0,0;

    E3 << 0,0,0,
          0,0,1,
          0,0,0;*/


    Eigen::Matrix3d mu2;
    mu2 = a_j*mu_j;

    Eigen::Matrix3d A,B,C,D,E,temp;
    temp = GroupMathSE::ExpMath::SE2_Adjoint(mu2.inverse());
    A =temp*cov_j*temp.transpose();
    B = cov_m;


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

    Eigen::Matrix3d cov_nin;
    cov_nin = A + B + C + D + E;

    
    Fusion(a_i,mu_i,cov_i,a_j,mu_j,cov_nin,mu_m,mu_i_bar,Sigma_i_bar);
    
}

