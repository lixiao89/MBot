#ifndef NEIGHBOR_INFO_SUB_HPP_
#define NEIGHBOR_INFO_SUB_HPP_


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose2D.h"

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
        


        //constructor
        NeighborInfoSub();

        // overloading constructor
        NeighborInfoSub(ros::NodeHandle& nh,std::string poseEstSubTopic)
        {
            nh_ = nh;
            
            // initialize subscriber and publisher
            infoSub_ = nh_.subscribe<geometry_msgs::Pose2D>(poseEstSubTopic, 1000, &NeighborInfoSub::neighborInfoCallback,this); 

        }

        ~NeighborInfoSub(void)
        {
        }



	void neighborInfoCallback(const geometry_msgs::Pose2D::ConstPtr& neighborPoseEst)
	{
	    subscribedX = neighborPoseEst->x;
        subscribedY = neighborPoseEst->y;
        subscribedTheta = neighborPoseEst->theta;
	} 

};      

#endif

