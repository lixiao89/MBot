#ifndef DIST_LOCALIZATION__HPP_
#define DIST_LOCALIZATION__HPP_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/LaserScan.h"
#include "ProcessLaserScan.hpp"
#include "PositionGT.hpp"
#include "ExpMath.cpp"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Pose.h"
#include "NeighborInfoSub.hpp"
#include "JointEncoderSub.hpp"
#include "mrl1_ros/ExpL.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Quaternion.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <sstream>
#include <vector>
#include <iostream>
#include <math.h>


using namespace std;
using namespace GroupMathSE;

// Inherits from class "ProcessLaserScan"
class DistLocalization : public ProcessLaserScan, public NeighborInfoSub, public JointEncoderSub{

 private:
        //used in function "SDEPrediction"
        ros::Time previousTime;

        // used in function "velocityPID"
        float previous_error;
        float integral_error;
//----------- ExpLocalization Member Variables ----------
        //initial position used in "expLocalization"
        Eigen::Matrix3d a_i;
        Eigen::Matrix3d mu_i;
        Eigen::Matrix3d cov_i;

        Eigen::Matrix3d a_j;
        //Eigen::Matrix3d mu_j;
       // Eigen::Matrix3d cov_j;

        Eigen::Matrix3d mu_m;
        Eigen::Matrix3d cov_m;

        Eigen::Matrix3d mu_i_bar;
        Eigen::Matrix3d Sigma_i_bar;


        float w1;
        float w2;
        
        Eigen::Vector3d neighborGT;
        Eigen::Vector3d selfGT;

        geometry_msgs::Pose2D estError;

    public:
        std_msgs::Float64 leftEffort;
        std_msgs::Float64 rightEffort;
        mrl1_ros::ExpL posEsti;
        geometry_msgs::Point rqtEst;

        ros::Publisher leftWheelPub_;
        ros::Publisher rightWheelPub_;
       
        ros::Publisher poseEstimate_;
        ros::Publisher rqtEst_;
        ros::NodeHandle nh_;
   
        ros::Subscriber neighborGT_;
        ros::Subscriber selfGT_;
        ros::Publisher estError_;

        std::string robot;

        // Constructor
        DistLocalization(ros::NodeHandle& nh, Eigen::Matrix3d initialPose,Eigen::Matrix3d neighborIP,std::string laserTopic, std::string leftwheelcontrollertopic, std::string rightwheelcontrollertopic, std::string poseEstPubTopic, std::string poseEstSubTopic,std::string jointStateTopic,std::string selfGTTopic,std::string neighborGTTopic,std::string errorTopic):ProcessLaserScan(nh,initialPose,neighborIP,laserTopic),NeighborInfoSub(nh,neighborIP,poseEstSubTopic),JointEncoderSub(nh,jointStateTopic)
        {
            nh_ = nh;

             leftWheelPub_ = nh_.advertise<std_msgs::Float64>(leftwheelcontrollertopic, 1000);
            rightWheelPub_ = nh_.advertise<std_msgs::Float64>(rightwheelcontrollertopic, 1000);  
            
            poseEstimate_ = nh_.advertise<mrl1_ros::ExpL>(poseEstPubTopic,1000);


            estError_ = nh_.advertise<geometry_msgs::Pose2D>(errorTopic,1000);

 neighborGT_ = nh_.subscribe<geometry_msgs::Pose>(neighborGTTopic, 1000, &DistLocalization::neighborGTCallback,this); 

 selfGT_ = nh_.subscribe<geometry_msgs::Pose>(selfGTTopic, 1000, &DistLocalization::selfGTCallback,this); 


            
            neighborGT = ExpMath::SE2ToXYTheta(neighborIP);
            selfGT = ExpMath::SE2ToXYTheta(initialPose);
            robot = laserTopic;

            previousTime = ros::Time::now();

            previous_error = 0;
            integral_error = 0;

            a_i = initialPose;

            cov_i << 0.25,0,0,
                     0,0.25,0,
                     0,0,0.25;


            a_j << 1,0,0,
                   0,1,0,
                   0,0,1;

            cov_m << 0.0001, 0, 0,
                     0, 0.0001, 0,
                     0, 0, 0.0001;
            w1 = 0.01;
            w2 = 0.01;

            mu_i_bar = Eigen::MatrixXd::Identity(3,3);
            Sigma_i_bar = mu_i_bar;
        }


        // Destructor
        ~DistLocalization(void)
        {
        }

        void neighborGTCallback(const geometry_msgs::Pose::ConstPtr& neighborGT_msg)
        {
            double theta;

            float x = neighborGT_msg->orientation.x;
            float y = neighborGT_msg->orientation.y;
            float z = neighborGT_msg->orientation.z;
            float w = neighborGT_msg->orientation.w;

            float mag = sqrt(pow(x,2)+pow(y,2)+pow(z,2)+pow(w,2));

            geometry_msgs::Quaternion temp;
            temp = neighborGT_msg->orientation;
 
             temp.x = temp.x/mag;
             temp.y = temp.y/mag;
             temp.z = temp.z/mag;
             temp.w = temp.w/mag;


            theta = tf::getYaw(temp);
            neighborGT << neighborGT_msg->position.x,neighborGT_msg->position.y,theta;
        }

 void selfGTCallback(const geometry_msgs::Pose::ConstPtr& selfGT_msg)
        {
            double theta;

            float x = selfGT_msg->orientation.x;
            float y = selfGT_msg->orientation.y;
            float z = selfGT_msg->orientation.z;
            float w = selfGT_msg->orientation.w;


          float mag = sqrt(pow(x,2)+pow(y,2)+pow(z,2)+pow(w,2));

            geometry_msgs::Quaternion temp;
            temp = selfGT_msg->orientation;
            
             temp.x = temp.x/mag;
             temp.y = temp.y/mag;
             temp.z = temp.z/mag;
             temp.w = temp.w/mag;




            theta = tf::getYaw(temp);


            selfGT << selfGT_msg->position.x,selfGT_msg->position.y,theta;
        }
	
        // PID control velocity via effort
        std_msgs::Float64 velocityPID(float velDesire, float currVel,float kp, float ki, float kd);

        // main method to control robot
        void moveToLongestScan();


        void moveSinasoidal();

        
        // publish motor commands
        void Explore(void)
        {
              
           // if(rangeReadings.size() < scanNum)
            if(encoderLeftVel > 10)
            { 
                ros::Duration(0.5).sleep(); 
            }
            else
            {

               //current exploration method! 
              
              //this->moveToLongestScan();    
              //this->moveSinasoidal();



                leftEffort = this->velocityPID(1,encoderLeftVel,0.5,0.4,0.2);
                rightEffort = this->velocityPID(1,encoderRightVel,0.5,0.4,0.2);
               leftWheelPub_.publish(leftEffort);
               rightWheelPub_.publish(rightEffort);
 
                cout<< leftEffort.data<<","<<rightEffort.data<<endl;
                //cout<< encoderLeftVel << endl;
            }

        }


//----------  Distributed Localization Methods  ---------
        

//////////////////  Distributed EKF //////////////////


        // Main method to call
        void distEKF();
       
       

        
        
///////////////// Exponential Localization ///////////
        
        // Main method to call
        void expLocalization();

        // given command of the two wheels, returns the predicted mean and covariance of the kinematic cart
        // inputs: w1 and w2 are left and right angular velocities
        // outputs: predicted mean and covariance in SE2
        void SDEPrediction(float w1, float w2, Eigen::Matrix3d& mu, Eigen::Matrix3d& cov);
 
  
        // returns xi and Si
         void generate_Si(Eigen::Matrix3d Sigma_i, Eigen::Vector3d& xi, Eigen::Matrix3d& Si);       
       
        // returns xm and Sm
         void generate_Sm(Eigen::Matrix3d m_im,Eigen::Matrix3d mu_m,Eigen::Matrix3d a_m,Eigen::Matrix3d a_i,Eigen::Matrix3d mu_i,Eigen::Matrix3d Sigma_m,Eigen::Vector3d& xm,Eigen::Matrix3d& Sm);

         // % mu_i and cov_i are 3x3 matrices representing the estimated mean
         // % and covariance of the robot to be updated
         //
         // % a_i is a 3x3 matrix denoting the initial position of robot i
         //
         // % MU_m, COV_m are 3x3xn matrices representing the means and covariances of the n neighboring robots for robot i to take measurements from
         //
         // % A_i is a 3x3 matrix denoting the initial positions of all other m robots
         // % Mim 3x3 is the exact measurement of robot i relative to m [xi,Si] = Generate_Si(cov_i)
         void Fusion(Eigen::Matrix3d a_i,Eigen::Matrix3d mu_i,Eigen::Matrix3d cov_i,Eigen::Matrix3d A_m,Eigen::Matrix3d MU_m,Eigen::Matrix3d COV_m,Eigen::Matrix3d Mim, Eigen::Matrix3d& mu_i_bar, Eigen::Matrix3d& Sigma_i_bar);

        // returns the final estimate update for mean (mu_i_bar) and covariance (Sigma_i_bar)
         void fusion_with_sensor_noise(Eigen::Matrix3d a_i,Eigen::Matrix3d mu_i,Eigen::Matrix3d cov_i,Eigen::Matrix3d a_j,Eigen::Matrix3d mu_j,Eigen::Matrix3d cov_j,Eigen::Matrix3d mu_m,Eigen::Matrix3d cov_m,Eigen::Matrix3d& mu_i_bar,Eigen::Matrix3d& Sigma_i_bar);


};

#endif


