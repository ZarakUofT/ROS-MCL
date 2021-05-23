#ifndef __PARTICLE_H__
#define __PARTICLE_H__

#include "base.h"
#include "odom.h"

struct Pose{
    double x, y, yaw;

    Pose(double pos_x, double pos_y, double yaw)
        :x(pos_x), y(pos_y), yaw(yaw){}
};

class Particle{
private:
    std::shared_ptr<Pose> pose;
    std::shared_ptr<OdomData> odomData;
    
    uint32_t mapPosX, mapPosY;
    static double alpha1, alpha2, alpha3, alpha4;
public:
    Particle(std::shared_ptr<OdomData> odom_data);
    Particle(std::shared_ptr<Pose> _pose, std::shared_ptr<OdomData> odom_data);
    ~Particle();

    static void initParticle(double a1, double a2, double a3, double a4);
    
    // Update funcs
    void update();
    void applyOdomMotionModel(Pose* newOdom);
    void applySensorModel();

};
#endif // __PARTICLE_H__