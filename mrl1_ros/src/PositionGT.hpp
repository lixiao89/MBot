#ifndef POSITION_GT_HPP_
#define POSITION_GT_HPP_


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

#include <sstream>
#include <vector>
#include <iostream>

// class subscribing to the position ground truth provided by Gazebo
class PositionGT{ 
	
    public:

       
        ros::NodeHandle nh_;
        ros::Subscriber GTscanSub_;
        
	    
        geometry_msgs::Pose posMrl1;
        geometry_msgs::Pose posMrl2;

        
        ros::Publisher posMrl1Pub_;
        ros::Publisher posMrl2Pub_;

        //constructor
        PositionGT();

        // overloading constructor
        PositionGT(ros::NodeHandle& nh,std::string modelStateTopic)
        {
            nh_ = nh;
            
            // initialize subscriber and publisher
            GTscanSub_ = nh_.subscribe<gazebo_msgs::ModelStates>(modelStateTopic, 1000, &PositionGT::GTCallback,this); 

            
            posMrl1Pub_ = nh_.advertise<geometry_msgs::Pose>("/mrl1Pos", 1000);
            posMrl2Pub_ = nh_.advertise<geometry_msgs::Pose>("/mrl2Pos", 1000);  
            
        }

        ~PositionGT(void)
        {
        }



     // Call back to obtain laser sensor data and store in members
     // "rangeReadings" and "angleReadings"
	void GTCallback(const gazebo_msgs::ModelStates::ConstPtr& modelstate_msg)
    {
        
        posMrl1 = modelstate_msg->pose[1];
        posMrl2 = modelstate_msg->pose[2];

    }


    void posGTPublish()
    {
        posMrl1Pub_.publish(posMrl1);
        posMrl2Pub_.publish(posMrl2);

    }
};      

#endif



