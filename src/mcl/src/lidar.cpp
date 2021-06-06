#include "lidar.h"

LidarData::LidarData() 
    : angleInc(0.f), rangeMin(0.f), rangeMax(0.f), ranges({}){
    
}

LidarData::LidarData(float angle_inc, float range_min, float range_max, std::vector<float> _ranges) 
    : angleInc(angle_inc), rangeMin(range_min), rangeMax(range_max), ranges(_ranges){
    
}

LidarData::~LidarData() {
    
}

void LidarData::callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    this->angleInc = msg->angle_increment;
    this->rangeMin = msg->range_min;
    this->rangeMax = msg->range_max;
    this->ranges = msg->ranges;
}
