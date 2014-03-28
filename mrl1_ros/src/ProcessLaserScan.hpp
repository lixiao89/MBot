#ifndef PROCESS_LASER_SCAN_HPP_
#define PROCESS_LASER_SCAN_HPP_


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/LaserScan.h"

#include <sstream>
#include <vector>
#include <iostream>
#include <tf/transform_broadcaster.h>

class ProcessLaserScan{ 
	
    public:

       
        ros::NodeHandle nh_;
        ros::Subscriber scanSub_;
        
        std::vector<float> rangeReadings;
	    std::vector<float> angleReadings;
	
        float maxRange;
        // number of laser scans
        int scanNum;
        

        //constructor
        ProcessLaserScan();

        // overloading constructor
        ProcessLaserScan(ros::NodeHandle& nh,std::string laserTopic)
        {
            nh_ = nh;
            
            // initialize subscriber and publisher
            scanSub_ = nh_.subscribe<sensor_msgs::LaserScan>(laserTopic, 1000, &ProcessLaserScan::laserScanCallback,this); 

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
       // std::cout<<Lscan_msg->angle_increment<<std::endl;
		}
		angleReadings.pop_back();
	} 

};      

#endif














