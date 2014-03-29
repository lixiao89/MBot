#include "DistLocalization.hpp"


void DistLocalization::moveToLongestScan()
{
    
    int minRangeIndex = 0;
    float minRangeTemp = maxRange; 

    for(int i=0; i < rangeReadings.size();i++)
    {
        if(rangeReadings.at(i) < minRangeTemp)
        {
            minRangeTemp = rangeReadings.at(i);
            minRangeIndex = i;
        }
    }           

    if(angleReadings.at(minRangeIndex) > 0)
    {
        leftEffort.data = 2;
        rightEffort.data = -2;
    }
    else if(angleReadings.at(minRangeIndex) < 0)
    {
        leftEffort.data = -2;
        rightEffort.data = 2;

    }
    else
    {
        leftEffort.data = 3;
        rightEffort.data = 3;
    }

}

void DistLocalization::moveSinasoidal()
{   
    ros::Time time = ros::Time::now();

    float t;
    t = time.toSec();

    leftEffort.data = sin(t)+1;
    rightEffort.data = sin(t)+0.5;
}

//////////////////// Distributed EKF ///////////////////////
//






///////////////// Exponential Localization /////////////////


