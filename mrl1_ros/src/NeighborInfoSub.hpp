#ifndef NEIGHBOR_INFO_SUB_HPP_
#define NEIGHBOR_INFO_SUB_HPP_


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose2D.h"
#include "mrl1_ros/ExpL.h"

#include <eigen3/Eigen/Dense>

#include <sstream>
#include <vector>
#include <iostream>

class NeighborInfoSub{ 
	
    public:

       
        ros::NodeHandle nh_;
        ros::Subscriber infoSub_;
        
	   float subscribedX;
       float subscribedY;
       float subscribedTheta;
        
       Eigen::Matrix3d subscribedCov;       
        
        //constructor
        NeighborInfoSub();

        // overloading constructor
        NeighborInfoSub(ros::NodeHandle& nh,Eigen::Matrix3d a_i,std::string poseEstSubTopic)
        {
            nh_ = nh;
            
            // initialize subscriber and publisher
            infoSub_ = nh_.subscribe<mrl1_ros::ExpL>(poseEstSubTopic, 1000, &NeighborInfoSub::neighborInfoCallback,this); 

            Eigen::Vector3d coord;
            coord = GroupMathSE::ExpMath::SE2ToXYTheta(a_i);
            subscribedX = coord(1-1);
            subscribedY = coord(2-1);
            subscribedTheta = coord(3-1);

            subscribedCov = 4*Eigen::MatrixXd::Identity(3,3);
        }

        ~NeighborInfoSub(void)
        {
        }



	void neighborInfoCallback(const mrl1_ros::ExpL::ConstPtr& neighborPoseEst)
	{
	    subscribedX = neighborPoseEst->poseEst.x;
        subscribedY = neighborPoseEst->poseEst.y;
        subscribedTheta = neighborPoseEst->poseEst.theta;


       subscribedCov << neighborPoseEst->cov[0],neighborPoseEst->cov[1],neighborPoseEst->cov[2],neighborPoseEst->cov[3],neighborPoseEst->cov[4],neighborPoseEst->cov[5],neighborPoseEst->cov[6],neighborPoseEst->cov[7],neighborPoseEst->cov[8];
       

	} 

};      

#endif

