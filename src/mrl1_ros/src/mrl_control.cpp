#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/LaserScan.h"
#include "ProcessLaserScan.hpp"
//include "motorControl.hpp"
#include "MoveToLongestScan.hpp"

#include <sstream>
#include <vector>



int main(int argc, char **argv)
{

  ros::init(argc, argv, "mrl_control");


  ros::NodeHandle n;
  
// Initializes control for robot "mrl1"
//inputs are: node handle, laser scan topic name, left wheel controller, right wheel controller
  MoveToLongestScan mctrl1(n,"/mrl1/laser/scan","/mrl1/left_wheel_controller/command","/mrl1/right_wheel_controller/command");
  
// Initializes control for robot "mrl2"
//inputs are: node handle, laser scan topic name, left wheel controller, right wheel controller
  MoveToLongestScan mctrl2(n,"/mrl2/laser/scan","/mrl2/left_wheel_controller/command","/mrl2/right_wheel_controller/command");
 
 MoveToLongestScan mctrl3(n,"/mrl3/laser/scan","/mrl3/left_wheel_controller/command","/mrl3/right_wheel_controller/command");
 
 //MoveToLongestScan mctrl4(n,"/mrl4/laser/scan","/mrl4/left_wheel_controller/command","/mrl4/right_wheel_controller/command");
 
// MoveToLongestScan mctrl5(n,"/mrl5/laser/scan","/mrl5/left_wheel_controller/command","/mrl5/right_wheel_controller/command");
 


  ros::Rate loop_rate(40);

  int count = 0;
    while (n.ok())
     {
        //control motion of robot "mrl1"
        mctrl1.publishMotorControl();
        mctrl2.publishMotorControl();

      mctrl3.publishMotorControl();
      //mctrl4.publishMotorControl();
      //mctrl5.publishMotorControl();
        ros::spinOnce();
        loop_rate.sleep();
        ++count;

     }
 
  return 0;
}


 


