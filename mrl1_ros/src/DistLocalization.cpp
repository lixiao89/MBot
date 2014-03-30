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
}
        // returns the final estimate update for mean (mu_i_bar) and covariance (Sigma_i_bar)
         void DistLocalization::fusion_with_sensor_noise(Eigen::Matrix3d a_i,Eigen::Matrix3d mu_i,Eigen::Matrix3d cov_i,Eigen::Matrix3d a_j,Eigen::Matrix3d mu_j,Eigen::Matrix3d cov_j,Eigen::Matrix3d mu_m,Eigen::Matrix3d cov_m,Eigen::Matrix3d& mu_i_bar,Eigen::Matrix3d& Sigma_i_bar)
{
}

