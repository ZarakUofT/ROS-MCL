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

    //private functions
    double inline computePHit(double zt, double zt_actual);
    double inline computePShort(double zt, double zt_actual);
    double inline computePMax(double zt);
    double inline computePrand(double zt);

    void applyOdomMotionModel(double delta_rot1, double delta_trans, double delta_rot2, 
                              double delta_rot1_noise, double delta_rot2_noise);
    void applySensorModel();

    static void init_static_vars();
public:
    Particle(double weight);
    Particle(std::shared_ptr<Pose> _pose, double weight, uint map_pos_x, uint map_pos_y);
    ~Particle();
    Particle (const Particle& p);

    static void setStatics(const std::shared_ptr<OdomData> odom_data, const std::shared_ptr<LidarData> lidar_data,
                          const std::shared_ptr<Map> _map);
    static void setOdomModel(double a1, double a2, double a3, double a4);
    static void setBeamModel(double z_hit, double z_max, double z_short, double z_rand, 
                            double sigma_hit, double lambda_short, double chi_outlier);
    
    // Update funcs
    void update(double delta_rot1, double delta_trans, double delta_rot2, 
                double delta_rot1_noise, double delta_rot2_noise);

    // getter funcs
    const std::shared_ptr<Pose> inline getPose() const {return this->pose;}
    const std::pair<uint, uint> inline getMapPos() {return std::make_pair(this->mapPosX, this->mapPosY);}
    double inline getWeight() const {return this->weight;}

};

//Initializing Class Static Vars
double Particle::alpha1 = 0.f;
double Particle::alpha2 = 0.f;
double Particle::alpha3 = 0.f;
double Particle::alpha4 = 0.f;

double Particle::zHit = 0.f;
double Particle::zMax = 0.f;
double Particle::zShort = 0.f;
double Particle::zRand = 0.f;
double Particle::sigmaHit = 0.f;
double Particle::lambdaShort = 0.f;
double Particle::chiOutlier = 0.f;


std::shared_ptr<OdomData> Particle::odomData = nullptr;
std::shared_ptr<LidarData> Particle::lidarData = nullptr;
std::shared_ptr<Map> Particle::map = nullptr;
#endif // __PARTICLE_H__