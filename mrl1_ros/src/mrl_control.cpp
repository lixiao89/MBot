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

   /* symbol t("t");
    ex f = 2*t+1;


    double a = 0;
    double b = 1;

    double res;

res = GroupMathSE::ExpMath::SimpsonIntegrate(t,f,4,b);

cout<< res << endl;*/

   

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

  Eigen::Matrix3d a_i1, a_i2;

  a_i1 << cos(0.5233),-sin(0.5233),1,
          sin(0.5233),cos(0.5233),1,
          0,0,1;

  a_i2 << cos(0.5233),-sin(0.5233),1,
          sin(0.5233),cos(0.5233),2,
          0,0,1;




// Initializes control for robot "mrl1"
//inputs are: node handle, laser scan topic name, left wheel controller, right wheel controller, published self pose estimate, subscribed neighbor pose estimate
  DistLocalization mctrl1(n,a_i1,a_i2,"/mrl1/laser/scan","/mrl2/laser/scan","/mrl1/left_wheel_controller/command","/mrl1/right_wheel_controller/command","/mrl1/poseEstPub","/mrl1/poseEstPubEKF","/mrl2/poseEstPub","/mrl2/poseEstPubEKF","/mrl1/joint_states","/mrl1Pos","/mrl2Pos","/mrl1/estError");

  
// Initializes control for robot "mrl2"
  DistLocalization mctrl2(n,a_i2,a_i1,"/mrl2/laser/scan","/mrl1/laser/scan","/mrl2/left_wheel_controller/command","/mrl2/right_wheel_controller/command","/mrl2/poseEstPub","/mrl2/poseEstPubEKF","/mrl1/poseEstPub","/mrl1/poseEstPubEKF","/mrl2/joint_states","/mrl2Pos","/mrl1Pos","/mrl2/estError");



  ros::Rate loop_rate(15);


/*for(int i=0;i<6;i++)
{
    mctrl1.Explore();
    mctrl2.Explore();
    ros::spinOnce();
    loop_rate.sleep();
}*/


     const char *pathexp1 = "/home/lixiao/Desktop/MBot/src/MBot/mrl1_ros/data/robot1.txt";
     ofstream ofsExp1(pathexp1);
     // const char *pathekf1 = "/home/lixiao/Desktop/MBot/src/MBot/mrl1_ros/data/robot1.txt";
      // ofstream ofsEKF1(pathekf1);
           
const char *pathexp2 = "/home/lixiao/Desktop/MBot/src/MBot/mrl1_ros/data/robot2.txt";
     ofstream ofsExp2(pathexp2);
    //  const char *pathekf2 = "/home/lixiao/Desktop/MBot/src/MBot/mrl1_ros/data/EKF2.txt";
    //   ofstream ofsEKF2(pathekf2);
      

  int count = 0;
    while (n.ok())
     {

        //control motion of robot "mrl1"
        mctrl1.Explore();
        mctrl2.Explore();


 //       mctrl1.distEKF();
 //       mctrl2.distEKF();

      mctrl1.expLocalization();
      mctrl2.expLocalization();
     
      
      ofsExp1<<mctrl1.currTime<<","<<mctrl1.currPose(0)<<","<<mctrl1.currPose(1)<<","<<mctrl1.currPose(2)<<","<<mctrl1.poseCurr(0)<<","<<mctrl1.poseCurr(1)<<","<<mctrl1.poseCurr(2)<<","<<mctrl1.selfGT(0)<<","<<mctrl1.selfGT(1)<<","<<mctrl1.selfGT(2)<<","<<mctrl1.poseDR(0)<<","<<mctrl1.poseDR(1)<<","<<mctrl1.poseDR(2)<<endl;

      ofsExp2<<mctrl2.currTime<<","<<mctrl2.currPose(0)<<","<<mctrl2.currPose(1)<<","<<mctrl2.currPose(2)<<","<<mctrl2.poseCurr(0)<<","<<mctrl2.poseCurr(1)<<","<<mctrl2.poseCurr(2)<<","<<mctrl2.selfGT(0)<<","<<mctrl2.selfGT(1)<<","<<mctrl2.selfGT(2)<<","<<mctrl2.poseDR(0)<<","<<mctrl2.poseDR(1)<<","<<mctrl2.poseDR(2)<<endl;




    // mctrl1.distEKF();
    //  mctrl2.distEKF();

//ofsEKF1<<mctrl1.currTime<<","<<mctrl1.poseCurr(0)<<","<<mctrl1.poseCurr(1)<<","<<mctrl1.poseCurr(2)<<","<<mctrl1.selfGT(0)<<","<<mctrl1.selfGT(1)<<","<<mctrl1.selfGT(2)<<endl;

//ofsEKF2<<mctrl2.currTime<<","<<mctrl2.poseCurr(0)<<","<<mctrl2.poseCurr(1)<<","<<mctrl2.poseCurr(2)<<","<<mctrl2.selfGT(0)<<","<<mctrl2.selfGT(1)<<","<<mctrl2.selfGT(2)<<endl;





        GTPub.posGTPublish();
        

        ros::spinOnce();
        loop_rate.sleep();
        ++count;

     }
 
ofsExp1.close();
ofsExp2.close();
  return 0;
}


 


