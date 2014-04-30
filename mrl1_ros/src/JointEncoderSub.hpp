#ifndef JOINT_ENCODER_SUB_HPP_
#define JOINT_ENCODER_SUB_HPP_


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Header.h"
#include "time.h"

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
        
       ros::Time currTime;


        //constructor
        JointEncoderSub();

        // overloading constructor
        JointEncoderSub(ros::NodeHandle& nh,std::string jointStateTopic)
        {
            nh_ = nh;
            encoderLeftVel = 0;
            encoderRightVel = 0;
            // initialize subscriber and publisher
            encoderSub_ = nh_.subscribe<sensor_msgs::JointState>(jointStateTopic, 1000, &JointEncoderSub::jointEncoderCallback,this); 
            
            currTime = ros::Time::now();
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


          if(encoderLeftVel > 10 || encoderRightVel >10 || encoderLeftVel < -10 || encoderRightVel < -10)
            {
                encoderLeftVel = 0.001;
                encoderRightVel = 0.001;
            }




        std_msgs::Header jointStateHeader;
       jointStateHeader = jointState_msg->header;

        currTime = jointStateHeader.stamp;
       // std::cout << encoderLeftVel << std::endl;
	} 

};      

#endif
