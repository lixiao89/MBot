#ifndef NEIGHBOR_INFO_SUB_EKF_HPP_
#define NEIGHBOR_INFO_SUB_EKF_HPP_


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose2D.h"
#include "mrl1_ros/ExpL.h"

#include <eigen3/Eigen/Dense>

#include <sstream>
#include <vector>
#include <iostream>

class NeighborInfoSubEKF{ 
	
    public:

       
        ros::NodeHandle nh_;
        ros::Subscriber infoSubEKF_;
        
	   float subscribedXEKF;
       float subscribedYEKF;
       float subscribedThetaEKF;
        
       Eigen::Matrix3d subscribedCovEKF;       
        
        //constructor
        NeighborInfoSubEKF();

        // overloading constructor
        NeighborInfoSubEKF(ros::NodeHandle& nh,Eigen::Matrix3d a_i,std::string poseEstSubTopicEKF)
        {
            nh_ = nh;
            
            // initialize subscriber and publisher
            infoSubEKF_ = nh_.subscribe<mrl1_ros::ExpL>(poseEstSubTopicEKF, 1000, &NeighborInfoSubEKF::neighborInfoCallbackEKF,this); 

            Eigen::Vector3d coord;
            coord = GroupMathSE::ExpMath::SE2ToXYTheta(a_i);
            subscribedXEKF = coord(1-1);
            subscribedYEKF = coord(2-1);
            subscribedThetaEKF = coord(3-1);

            subscribedCovEKF = 4*Eigen::MatrixXd::Identity(3,3);
        }

        ~NeighborInfoSubEKF(void)
        {
        }



	void neighborInfoCallbackEKF(const mrl1_ros::ExpL::ConstPtr& neighborPoseEstEKF)
	{
	    subscribedXEKF = neighborPoseEstEKF->poseEst.x;
        subscribedYEKF = neighborPoseEstEKF->poseEst.y;
        subscribedThetaEKF = neighborPoseEstEKF->poseEst.theta;


       subscribedCovEKF << neighborPoseEstEKF->cov[0],neighborPoseEstEKF->cov[1],neighborPoseEstEKF->cov[2],neighborPoseEstEKF->cov[3],neighborPoseEstEKF->cov[4],neighborPoseEstEKF->cov[5],neighborPoseEstEKF->cov[6],neighborPoseEstEKF->cov[7],neighborPoseEstEKF->cov[8];
       

	} 

};      

#endif

