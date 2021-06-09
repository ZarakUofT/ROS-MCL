#ifndef __PARTICLE_H__
#define __PARTICLE_H__

#include "odom.h"
#include "lidar.h"
#include "map.h"

class Particle{
private:
    std::shared_ptr<Pose> pose;
    uint mapPosX, mapPosY;
    double weight;

    // Static Variables
    static std::shared_ptr<OdomData> odomData;
    static std::shared_ptr<LidarData> lidarData;
    static std::shared_ptr<Map> map; // all particles share the same map
    static double alpha1, alpha2, alpha3, alpha4;
    static double zHit, zMax, zShort, zRand, sigmaHit, lambdaShort, chiOutlier;
public:
    Particle(double weight);
    Particle(std::shared_ptr<Pose> _pose, double weight, uint map_pos_x, uint map_pos_y);
    ~Particle();

    static void setStatics(const std::shared_ptr<OdomData> odom_data, const std::shared_ptr<LidarData> lidar_data,
                          const std::shared_ptr<Map> _map);
    static void setOdomModel(double a1, double a2, double a3, double a4);
    static void setBeamModel(double z_hit, double z_max, double z_short, double z_rand, 
                            double sigma_hit, double lambda_short, double chi_outlier);
    
    // Update funcs
    void update();
    void applyOdomMotionModel();
    void applySensorModel();

    double inline computePHit(double zt, double zt_actual);
    double inline computePShort(double zt, double zt_actual);
    double inline computePMax(double zt);
    double inline computePrand(double zt);

    // getter funcs
    const std::shared_ptr<Pose> inline getPose() const {return this->pose;}
    std::pair<uint32_t, uint32_t> inline getMapPos() {return std::make_pair(this->mapPosX, this->mapPosY);}
    double inline getWeight() const {return this->weight;}

};
#endif // __PARTICLE_H__