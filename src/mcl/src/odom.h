#ifndef __ODOM_H__
#define __ODOM_H__

#include "base.h"

typedef struct odom{
    double x, y, yaw;

    odom(double pos_x, double pos_y, double yaw)
        :x(pos_x), y(pos_y), yaw(yaw){}
} odom_t;

class OdomData{
    friend class Particle;
private:
    std::shared_ptr<odom_t> delta;
    std::shared_ptr<odom_t> currOdom;
public:
    OdomData();
    OdomData(std::shared_ptr<odom_t> curr_odom);
    OdomData(std::shared_ptr<odom_t> delt, std::shared_ptr<odom_t> currOdom);
    virtual ~OdomData();

    void odomCallback(const nav_msgs::Odometry::ConstPtr & msg);
};

#endif // __ODOM_H__