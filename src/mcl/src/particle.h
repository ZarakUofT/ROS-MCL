#ifndef __PARTICLE_H__
#define __PARTICLE_H__

#include "math.h"

// ROS Includes
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <matplot/matplot.h>

struct Pose{
    double x, y, yaw;

    Pose(double pos_x, double pos_y, double yaw)
        :x(pos_x), y(pos_y), yaw(yaw){}
};

class Particle{
private:
    Pose* pose;
    Pose* prevOdom;
    uint32_t mapPosX, mapPosY;
    static double alpha1, alpha2, alpha3, alpha4;
public:
    Particle(Pose _pose, Pose prev_pose);
    ~Particle();

    static void initParticle(double a1, double a2, double a3, double a4);
    
    // Update funcs
    void update();
    void applyOdomMotionModel(Pose* newOdom);

};
#endif // __PARTICLE_H__