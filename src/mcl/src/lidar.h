#ifndef __LIDAR_H__
#define __LIDAR_H__

#include "base.h"

class LidarData{
private:
    
public:
    float angleInc;
    float rangeMin;
    float rangeMax;
    std::vector<float> ranges;

    LidarData();
    LidarData(float angle_inc, float range_min, float range_max, std::vector<float> _ranges);
    virtual ~LidarData();

    void callback(const sensor_msgs::LaserScan::ConstPtr& msg);
};
#endif // __LIDAR_H__