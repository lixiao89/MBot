#ifndef DIST_LOCALIZATION__HPP_
#define DIST_LOCALIZATION__HPP_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/LaserScan.h"
#include "ProcessLaserScan.hpp"
#include <eigen3/Eigen/Dense>
#include <sstream>
#include <vector>
#include <iostream>


// Let robot turn to the left if longest scan is to the left of the robot's center and likewise to the right
//
// Inherits from class "ProcessLaserScan"
class DistLocalization : public ProcessLaserScan{

    public:
        std_msgs::Float64 leftEffort;
        std_msgs::Float64 rightEffort;

        ros::Publisher leftWheelPub_;
        ros::Publisher rightWheelPub_;
       
        ros::NodeHandle nh_;


        // Constructor
        DistLocalization(ros::NodeHandle& nh,std::string laserTopic, std::string leftwheelcontrollertopic, std::string rightwheelcontrollertopic):ProcessLaserScan(nh,laserTopic)
        {
            nh_ = nh;

             leftWheelPub_ = nh_.advertise<std_msgs::Float64>(leftwheelcontrollertopic, 1000);
            rightWheelPub_ = nh_.advertise<std_msgs::Float64>(rightwheelcontrollertopic, 1000);  
            

        }


        // Destructor
        ~DistLocalization()
        {
        }

        
        // main method to control robot
        void moveToLongestScan()
       {
        }


        // publish motor commands
        void Explore()
        {
              
            if(rangeReadings.size() < scanNum)
            { 
                ros::Duration(0.5).sleep(); 
            }
            else
            {

               //current exploration method! 
               moveToLongestScan();    
               leftWheelPub_.publish(leftEffort);
               rightWheelPub_.publish(rightEffort);
 
            // std::cout<<angleReadings.at(1)<<std::endl;
            }

        }


//----------  Distributed Localization Methods  ---------
        

        // Distributed EKF
        void distEKF()
        {
        }
};

#endif



























