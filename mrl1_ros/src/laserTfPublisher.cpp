#include "ros/ros.h"
#include "std_msgs/String.h"


// intended to publish the laser scans as tfs relative to the base frame

int main(int argc, char **argv)
{
    ros::init(argc,argv,"laserTfPublisher");


    ros::NodeHandle n;
  
    ros::Subscriber scan_sub = n.subscribe("mrl/laser/scan", 1000, &ProcessLaserScan::laserScanCallback,&LScan); // do I need to spin here?


}
