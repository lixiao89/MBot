#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/LaserScan.h"
#include "ProcessLaserScan.hpp"
#include "DistLocalization.cpp"
#include "PositionGT.hpp"
#include "eigen3/Eigen/Dense"
#include "ExpMath.cpp"

#include <sstream>
#include <vector>


using namespace std;

int main(int argc, char **argv)
{

    Eigen::Matrix3d X;
    X << 0,-1,0,
         1,0,1,
         0,0,1;

    cout<<"ad of X is:"<<endl<<GroupMathSE::ExpMath::SE2_Adjoint(X)<<endl;

  ros::init(argc, argv, "mrl_control");


  ros::NodeHandle n;
  

  // used to publish the ground truth of the poses of the robots given by gazebo
  PositionGT GTPub(n,"/gazebo/model_states");

// Initializes control for robot "mrl1"
//inputs are: node handle, laser scan topic name, left wheel controller, right wheel controller
  DistLocalization mctrl1(n,"/mrl1/laser/scan","/mrl1/left_wheel_controller/command","/mrl1/right_wheel_controller/command");
  
// Initializes control for robot "mrl2"
//inputs are: node handle, laser scan topic name, left wheel controller, right wheel controller
  DistLocalization mctrl2(n,"/mrl2/laser/scan","/mrl2/left_wheel_controller/command","/mrl2/right_wheel_controller/command");
 
  DistLocalization mctrl3(n,"/mrl3/laser/scan","/mrl3/left_wheel_controller/command","/mrl3/right_wheel_controller/command");
 
 //MoveToLongestScan mctrl4(n,"/mrl4/laser/scan","/mrl4/left_wheel_controller/command","/mrl4/right_wheel_controller/command");
 
// MoveToLongestScan mctrl5(n,"/mrl5/laser/scan","/mrl5/left_wheel_controller/command","/mrl5/right_wheel_controller/command");
 


  ros::Rate loop_rate(40);

  int count = 0;
    while (n.ok())
     {
        //control motion of robot "mrl1"
        mctrl1.Explore();
        mctrl2.Explore();

        mctrl3.Explore();
      //mctrl4.publishMotorControl();
      //mctrl5.publishMotorControl();
        
        GTPub.posGTPublish();
        
        ros::spinOnce();
        loop_rate.sleep();
        ++count;

     }
 
  return 0;
}


 


