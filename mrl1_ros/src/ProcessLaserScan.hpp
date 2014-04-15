#ifndef PROCESS_LASER_SCAN_HPP_
#define PROCESS_LASER_SCAN_HPP_


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/LaserScan.h"
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <sstream>
#include <vector>
#include <iostream>
#include <tf/transform_broadcaster.h>

using namespace std;

class ProcessLaserScan{ 
	
    public:

       
        ros::NodeHandle nh_;
        ros::Subscriber scanSub_;
        
        std::vector<float> rangeReadings;
	    std::vector<float> angleReadings;
	

        float maxRange;
        float minRange;

        float minAngle;
        // number of laser scans
        int scanNum;
        

        //constructor
        ProcessLaserScan();

        // overloading constructor
        ProcessLaserScan(ros::NodeHandle& nh,Eigen::Matrix3d initialPose, Eigen::Matrix3d neighborIP,std::string laserTopic)
        {
            nh_ = nh;
            
            // initialize subscriber and publisher
            scanSub_ = nh_.subscribe<sensor_msgs::LaserScan>(laserTopic, 1000, &ProcessLaserScan::laserScanCallback,this); 
            Eigen::Matrix3d relPose;

            relPose = initialPose.inverse()*neighborIP;

            minAngle = atan2(relPose(2-1,1-1),relPose(2-1,2-1));

            minRange = sqrt(pow(relPose(1-1,3-1),2)+pow(relPose(2-1,3-1),2));
        }

        ~ProcessLaserScan(void)
        {
        }



     // Call back to obtain laser sensor data and store in members
     // "rangeReadings" and "angleReadings"
	void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& Lscan_msg)
	{
		rangeReadings.clear();
		angleReadings.clear();
	  size_t num_ranges = Lscan_msg->ranges.size();
      scanNum = num_ranges;


      maxRange = Lscan_msg->range_max;

	  int x;
	 
	 angleReadings.push_back(Lscan_msg->angle_min); //push the angle (in radians) of each ray into the vector
	 
	 
	  for (x = 0; x < num_ranges; x++)
	   { 
          
		rangeReadings.push_back(Lscan_msg->ranges[x]);
		angleReadings.push_back(angleReadings.back()+Lscan_msg->angle_increment);

         if(Lscan_msg->ranges[x] <= minRange)
           {
               minRange = rangeReadings.back();
               minAngle = angleReadings.back();
                
           }
		   
		}
		angleReadings.pop_back();

 //       cout<< "minRange:"<<minRange<<","<<"minAngle:"<<minAngle<<endl;

	} 



};      

#endif
