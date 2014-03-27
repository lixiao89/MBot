#include <DistLocalization.hpp>


DisLocalization::moveToLongestScan()
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
