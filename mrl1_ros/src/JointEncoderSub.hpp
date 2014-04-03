#ifndef JOINT_ENCODER_SUB_HPP_
#define JOINT_ENCODER_SUB_HPP_


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"

#include <sstream>
#include <vector>
#include <iostream>

class JointEncoderSub{ 
	
    public:

       
        ros::NodeHandle nh_;
        ros::Subscriber encoderSub_;
        
	   float encoderLeftVel;
       float encoderRightVel;
       float encoderLeftPos;
       float encoderRightPos;
        


        //constructor
        JointEncoderSub();

        // overloading constructor
        JointEncoderSub(ros::NodeHandle& nh,std::string jointStateTopic)
        {
            nh_ = nh;
            
            // initialize subscriber and publisher
            encoderSub_ = nh_.subscribe<sensor_msgs::JointState>(jointStateTopic, 1000, &JointEncoderSub::jointEncoderCallback,this); 

        }

        ~JointEncoderSub(void)
        {
        }



	void jointEncoderCallback(const sensor_msgs::JointState::ConstPtr& jointState_msg)
	{
        encoderLeftPos = jointState_msg->position[0];
        encoderRightPos = jointState_msg->position[1];

        encoderLeftVel = jointState_msg->velocity[0];
        encoderRightVel = jointState_msg->velocity[1];

       // std::cout << encoderLeftVel << std::endl;
	} 

};      

#endif
