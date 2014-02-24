#ifndef MOTOR_CONTROL_HPP_
#define MOTOR_CONTROL_HPP_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include "ProcessLaserScan.hpp"
#include "MoveToLongestScan.hpp"

#include <sstream>
#include <vector>
#include <iostream>


class motorControl{

    public:

       
        ros::NodeHandle nh_;
        ros::Subscriber scanSub_;
        ros::Publisher leftWheelPub_;
        ros::Publisher rightWheelPub_;
       
        std::vector<float> rangeReadings;
	    std::vector<float> angleReadings;
	
     
        // number of laser scans
        int scanNum;
        
        // initializes the controller method
        MoveToLongestScan controller;

        motorControl(ros::NodeHandle& nh,std::string laserTopic, std::string leftwheelcontrollertopic, std::string rightwheelcontrollertopic)
        {
            nh_ = nh;
            
            // initialize subscriber and publisher
            scanSub_ = nh_.subscribe<sensor_msgs::LaserScan>(laserTopic, 1000, &motorControl::controlCb,this); 
            

            leftWheelPub_ = nh_.advertise<std_msgs::Float64>(leftwheelcontrollertopic, 1000);
            rightWheelPub_ = nh_.advertise<std_msgs::Float64>(rightwheelcontrollertopic, 1000);  
            

            

        }

        ~motorControl(void)
        {
        }



     // Call back to obtain laser sensor data and store in members
     // "rangeReadings" and "angleReadings"
	void controlCb(const sensor_msgs::LaserScan::ConstPtr& Lscan_msg)
	{
		rangeReadings.clear();
		angleReadings.clear();
	  size_t num_ranges = Lscan_msg->ranges.size();
      scanNum = num_ranges;

	  int x;
	 
	 angleReadings.push_back(Lscan_msg->angle_min); //push the angle (in radians) of each ray into the vector
	 
	 
	  for (x = 0; x < num_ranges; x++)
	   {
		   
		rangeReadings.push_back(Lscan_msg->ranges[x]);
		angleReadings.push_back(angleReadings.back()+Lscan_msg->angle_increment);
		
		}
		angleReadings.pop_back();
           
	} 

 

    // publishes  the output from the controller class to the motor joints
    void motorController()
    {
        if(rangeReadings.size() < scanNum)
        { 
            ros::Duration(0.5).sleep(); 
        }
        else
        {

         controller.calculateMotorInputs(angleReadings, rangeReadings);
            
         leftWheelPub_.publish(controller.leftEffort);
         rightWheelPub_.publish(controller.rightEffort);
 
        }
       
    }
};

#endif


