#include "lidar.h"

LidarData::LidarData(ros::NodeHandle& n) 
    : nh(n), angleInc(0.f), rangeMin(0.f), rangeMax(0.f), ranges({})
{
    ros::Subscriber lidarSub = this->nh.subscribe("scan", 10, &LidarData::callback, this);
}

LidarData::LidarData(ros::NodeHandle& n,
    float angle_inc, float range_min, float range_max, std::vector<float> _ranges) 
    : nh(n), angleInc(angle_inc), rangeMin(range_min), rangeMax(range_max), ranges(_ranges)
{
    ros::Subscriber lidarSub = this->nh.subscribe("scan", 10, &LidarData::callback, this);
}

LidarData::~LidarData() {
    
}

void LidarData::callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    this->angleInc = msg->angle_increment;
    this->rangeMin = msg->range_min;
    this->rangeMax = msg->range_max;
    this->ranges = msg->ranges;
}
