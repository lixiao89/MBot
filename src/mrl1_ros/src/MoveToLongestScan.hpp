#ifndef MOVE_TO_LONGEST_SCAN_HPP_
#define MOVE_TO_LONGEST_SCAN_HPP_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/LaserScan.h"
#include "ProcessLaserScan.hpp"
#include </home/lixiao/Eigen3/Eigen/Dense>

#include <sstream>
#include <vector>
#include <iostream>


// Let robot turn to the left if longest scan is to the left of the robot's center and likewise to the right
//
// Inherits from class "ProcessLaserScan"
class MoveToLongestScan : public ProcessLaserScan{

    public:
        std_msgs::Float64 leftEffort;
        std_msgs::Float64 rightEffort;

        ros::Publisher leftWheelPub_;
        ros::Publisher rightWheelPub_;
       
        ros::NodeHandle nh_;


        // Constructor
        MoveToLongestScan(ros::NodeHandle& nh,std::string laserTopic, std::string leftwheelcontrollertopic, std::string rightwheelcontrollertopic):ProcessLaserScan(nh,laserTopic)
        {
            nh_ = nh;

             leftWheelPub_ = nh_.advertise<std_msgs::Float64>(leftwheelcontrollertopic, 1000);
            rightWheelPub_ = nh_.advertise<std_msgs::Float64>(rightwheelcontrollertopic, 1000);  
            

        }


        // Destructor
        ~MoveToLongestScan()
        {
        }

        
        // main method to control robot
        void calculateMotorControl()
        {
            int maxRangeIndex = 0;
            float maxRange = 0; 

            for(int i=0; i < rangeReadings.size();i++)
            {
                if(rangeReadings.at(i) > maxRange)
                {
                    maxRange = rangeReadings.at(i);
                    maxRangeIndex = i;
                }
            }           

            if(angleReadings.at(maxRangeIndex) > 0)
            {
                leftEffort.data = 2;
                rightEffort.data = 1;
            }
            else
            {
                leftEffort.data = 1;
                rightEffort.data = 2;

            }
        }


        // publish motor commands
        void publishMotorControl()
        {
              
            if(rangeReadings.size() < scanNum)
            { 
                ros::Duration(0.5).sleep(); 
            }
            else
            {

             calculateMotorControl();
             leftWheelPub_.publish(leftEffort);
             rightWheelPub_.publish(rightEffort);
 
            // std::cout<<angleReadings.at(1)<<std::endl;
            }

        }
};

#endif
