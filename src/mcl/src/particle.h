#ifndef __PARTICLE_H__
#define __PARTICLE_H__

#include "base.h"
#include "odom.h"
#include "lidar.h"
#include "map.h"

struct Pose{
    double x, y, yaw;

    Pose()
        :x(0.f), y(0.f), yaw(0.f){}
    Pose(double pos_x, double pos_y, double yaw)
        :x(pos_x), y(pos_y), yaw(yaw){}
};

class Particle{
private:
    std::shared_ptr<Pose> pose;
    std::shared_ptr<OdomData> odomData;
    std::shared_ptr<LidarData> lidarData;
    std::shared_ptr<Map> map; 

    uint32_t mapPosX, mapPosY;
    static double alpha1, alpha2, alpha3, alpha4;
    static double zHit, zMax, zShort, zRand, sigmaHit, lambdaShort, chiOutlier; 
public:
    Particle(std::shared_ptr<OdomData> odom_data, std::shared_ptr<LidarData> lidar_data, 
             std::shared_ptr<Map> _map);
    Particle(std::shared_ptr<Pose> _pose, std::shared_ptr<OdomData> odom_data, 
             std::shared_ptr<LidarData> lidar_data, std::shared_ptr<Map> _map);
    ~Particle();

    static void setOdomModel(double a1, double a2, double a3, double a4);
    static void setBeamModel(double z_hit, double z_max, double z_short, double z_rand, 
                            double sigma_hit, double lambda_short, double chi_outlier);
    
    // Update funcs
    void update();
    void applyOdomMotionModel();
    void applySensorModel();

};
#endif // __PARTICLE_H__