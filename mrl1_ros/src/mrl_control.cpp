#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/LaserScan.h"
#include "ProcessLaserScan.hpp"
#include "DistLocalization.cpp"
#include "PositionGT.hpp"
#include "eigen3/Eigen/Dense"

#include <sstream>
#include <vector>


using namespace std;

int main(int argc, char **argv)
{
  /* symbol x("x"), y("y");
             ex foo = x+y/2;
             ex bar = foo.subs(x==1);
             ex foobar = bar.subs(y==1.2);
             cout<< foobar<<endl;

             double m = 1.29;
             cout<<foobar+m<<endl;*/

    Eigen::Matrix3d X;
 double Ei[3][3][3]=
    {
        {
            {0,-1,0},
            {1,0,0},
            {0,0,0},
        },

        {
            {0,0,1},
            {0,0,0},
            {0,0,0},
        },

        {
            {0,0,0},
            {0,0,1},
            {0,0,0},
        },

    };

 X = GroupMathSE::ExpMath::mapArray3D(Ei,0);
    cout<<"ad of X is:"<<endl<<X<<endl;

  ros::init(argc, argv, "mrl_control");


  ros::NodeHandle n;
  

  // used to publish the ground truth of the poses of the robots given by gazebo
  PositionGT GTPub(n,"/gazebo/model_states");

// Initializes control for robot "mrl1"
//inputs are: node handle, laser scan topic name, left wheel controller, right wheel controller, published self pose estimate, subscribed neighbor pose estimate
  DistLocalization mctrl1(n,"/mrl1/laser/scan","/mrl1/left_wheel_controller/command","/mrl1/right_wheel_controller/command","/mrl1/poseEstPub","/mrl2/poseEstPub","/mrl1/joint_states");
  
// Initializes control for robot "mrl2"
  DistLocalization mctrl2(n,"/mrl2/laser/scan","/mrl2/left_wheel_controller/command","/mrl2/right_wheel_controller/command","/mrl2/poseEstPub","/mrl1/poseEstPub","/mrl2/joint_states");
 

  ros::Rate loop_rate(40);

  int count = 0;
    while (n.ok())
     {
        //control motion of robot "mrl1"
        mctrl1.Explore();
        mctrl2.Explore();


        mctrl1.expLocalization();
        mctrl2.expLocalization();

        GTPub.posGTPublish();
        

        ros::spinOnce();
        loop_rate.sleep();
        ++count;

     }
 
  return 0;
}


 


