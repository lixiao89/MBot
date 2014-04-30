#ifndef PROCESS_NEIGHBOR_LASER_SCAN_HPP_
#define PROCESS_NEIGHBOR_LASER_SCAN_HPP_


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

class ProcessNeighborLaserScan{ 
	
    public:

       
        ros::NodeHandle nh_;
        ros::Subscriber nscanSub_;
        
        std::vector<float> nrangeReadings;
	    std::vector<float> nangleReadings;
	

        float nmaxRange;
        float nminRange;

        float nminAngle;
        // number of laser scans
        int nscanNum;
        

        //constructor
        ProcessNeighborLaserScan();

        // overloading constructor
        ProcessNeighborLaserScan(ros::NodeHandle& nh,Eigen::Matrix3d initialPose, Eigen::Matrix3d neighborIP,std::string neighborLaserTopic)
        {
            nh_ = nh;
            
            // initialize subscriber and publisher
           nscanSub_ = nh_.subscribe<sensor_msgs::LaserScan>(neighborLaserTopic, 1000, &ProcessNeighborLaserScan::neighborLaserScanCallback,this); 
           
            Eigen::Matrix3d nrelPose;

            nrelPose = neighborIP.inverse()*initialPose;

            nminAngle = atan2(nrelPose(2-1,1-1),nrelPose(2-1,2-1));

            nminRange = sqrt(pow(nrelPose(1-1,3-1),2)+pow(nrelPose(2-1,3-1),2));
        }

        ~ProcessNeighborLaserScan(void)
        {
        }



     // Call back to obtain laser sensor data and store in members
     // "rangeReadings" and "angleReadings"
	void neighborLaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& NLscan_msg)
	{
		nrangeReadings.clear();
		nangleReadings.clear();
	  size_t nnum_ranges = NLscan_msg->ranges.size();
      nscanNum = nnum_ranges;


      nmaxRange = NLscan_msg->range_max;

	  int x;
	 
	 nangleReadings.push_back(NLscan_msg->angle_min); //push the angle (in radians) of each ray into the vector
	 
	 
	  for (x = 0; x < nnum_ranges; x++)
	   { 
          
		nrangeReadings.push_back(NLscan_msg->ranges[x]);
		nangleReadings.push_back(nangleReadings.back()+NLscan_msg->angle_increment);

         if(NLscan_msg->ranges[x] <= nminRange)
           {
               nminRange = nrangeReadings.back();
               nminAngle = nangleReadings.back();
                
           }
		   
		}
		nangleReadings.pop_back();

 //       cout<< "minRange:"<<minRange<<","<<"minAngle:"<<minAngle<<endl;

	} 



};      

#endif
