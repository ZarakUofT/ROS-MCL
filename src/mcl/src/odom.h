#ifndef __ODOM_H__
#define __ODOM_H__

#include "base.h"

typedef struct odom{
    double x, y, yaw;

    odom(double pos_x, double pos_y, double yaw)
        :x(pos_x), y(pos_y), yaw(yaw){}
} odom_t;

class OdomData{
private:
    ros::NodeHandle& nh;
    ros::Subscriber odomSuscriber;
public:
    std::shared_ptr<odom_t> delta;
    std::shared_ptr<odom_t> currOdom;

    OdomData(ros::NodeHandle& n);
    OdomData(ros::NodeHandle& n, std::shared_ptr<odom_t> curr_odom);
    OdomData(ros::NodeHandle& n, 
            std::shared_ptr<odom_t> delt, std::shared_ptr<odom_t> currOdom);
    virtual ~OdomData();

    void callback(const nav_msgs::Odometry::ConstPtr & msg);
};

#endif // __ODOM_H__