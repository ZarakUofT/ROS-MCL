#include "particle.h"

Particle::Particle(std::shared_ptr<OdomData> odom_data, std::shared_ptr<LidarData> lidar_data, 
                    std::shared_ptr<Map> _map) 
    : pose(std::make_shared<Pose>()), odomData(odom_data), lidarData(lidar_data), map(_map){}

Particle::Particle(std::shared_ptr<Pose> _pose, std::shared_ptr<OdomData> odom_data, 
                    std::shared_ptr<LidarData> lidar_data, std::shared_ptr<Map> _map) 
    : pose(_pose), odomData(odom_data), lidarData(lidar_data), map(_map){
}

Particle::~Particle() 
{
    
}

void Particle::setOdomModel(double a1, double a2, double a3, double a4) 
{
    alpha1 = a1;
    alpha2 = a2;
    alpha3 = a3;
    alpha4 = a4;
}

void Particle::setBeamModel(double z_hit, double z_max, double z_short, double z_rand, 
                            double sigma_hit, double lambda_short, double chi_outlier)
{
    zHit = z_hit;
    zMax = z_max;
    zShort = z_short;
    zRand = z_rand;
    sigmaHit = sigma_hit;
    lambdaShort = lambda_short;
    chiOutlier = chi_outlier;
}

void Particle::update() 
{
    
}

void Particle::applyOdomMotionModel() 
{    
    double delta_rot1, delta_trans, delta_rot2;
    double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;

    delta_rot1 = Math::angle_diff(atan2(this->odomData->delta->y, this->odomData->delta->x), this->odomData->currOdom->yaw);
    delta_trans = sqrt(pow((this->odomData->delta->x), 2) + pow((this->odomData->delta->y), 2));
    delta_rot2 = Math::angle_diff(Math::angle_diff(this->odomData->currOdom->yaw, this->odomData->currOdom->yaw), delta_rot1);

    delta_rot1_hat = Math::angle_diff(delta_rot1,
                     Math::sample_normal_dist(0, this->alpha1 * delta_rot1 + 
                            this->alpha2 * delta_trans));
    delta_trans_hat = delta_trans - 
                      Math::sample_normal_dist(0, this->alpha3*delta_trans +
                            this->alpha4*delta_rot1 +
                            this->alpha4*delta_rot2);
    delta_rot2_hat = Math::angle_diff(delta_rot2,
                     Math::sample_normal_dist(0, this->alpha1 * delta_rot2 + 
                            this->alpha2 * delta_trans));

    this->pose->x += delta_trans_hat * cos(this->pose->yaw + delta_rot1_hat);
    this->pose->y += delta_trans_hat * sin(this->pose->yaw + delta_rot1_hat);
    this->pose->yaw += delta_rot1_hat + delta_rot2_hat;

}

void Particle::applySensorModel() 
{
    
}
