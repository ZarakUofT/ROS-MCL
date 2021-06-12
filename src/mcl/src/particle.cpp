#include "particle.h"

Particle::Particle(double _weight) 
    : pose(std::make_shared<Pose>()), mapPosX(0), mapPosY(0), weight(_weight)
{
}

Particle::Particle(std::shared_ptr<Pose> _pose, double _weight, uint32_t map_pos_x, uint32_t map_pos_y) 
    : pose(_pose), mapPosX(map_pos_x), mapPosY(map_pos_y), weight(_weight)
{
}

Particle::~Particle() 
{
    
}

void Particle::setStatics(const std::shared_ptr<OdomData> odom_data, const std::shared_ptr<LidarData> lidar_data,
                          const std::shared_ptr<Map> _map) 
{
    Particle::odomData = odom_data;
    Particle::lidarData = lidar_data;
    Particle::map = _map;
}

void Particle::setOdomModel(double a1, double a2, double a3, double a4) 
{
    Particle::alpha1 = a1;
    Particle::alpha2 = a2;
    Particle::alpha3 = a3;
    Particle::alpha4 = a4;
}

void Particle::setBeamModel(double z_hit, double z_max, double z_short, double z_rand, 
                            double sigma_hit, double lambda_short, double chi_outlier)
{
    Particle::zHit = z_hit;
    Particle::zMax = z_max;
    Particle::zShort = z_short;
    Particle::zRand = z_rand;
    Particle::sigmaHit = sigma_hit;
    Particle::lambdaShort = lambda_short;
    Particle::chiOutlier = chi_outlier;
}

void Particle::update() 
{
    this->applyOdomMotionModel();
    this->applySensorModel();
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

    // determine position of the 2D grip map
    int row, col;

    row = this->map->getRefRow() + static_cast<int>(this->pose->x / Particle::map->getCellSize());
    col = this->map->getRefCol() + static_cast<int>(this->pose->y / Particle::map->getCellSize());

    // make sure this doesn't exceed the map size???
    this->mapPosX = (row >= 0) ? row : 0;
    this->mapPosY = (col >= 0) ? col : 0;
}

void Particle::applySensorModel() 
{
    double q  = 1.0;
    double p = 0.0;
    double zt = 0.0;
    int j = 0;
    double z_actual = 0.0;
    double angle = this->pose->yaw;
    
    // Note: the following computations do not get normalized, might run into issues later
    for (int i = 0; i < map->getLaserBeams(); i++) { // make sure we do this based on the recorded data in map
        z_actual = map->getActualRange(this->mapPosX, this->mapPosY, angle);
        zt = this->lidarData->ranges[j];
        p = this->zHit   * this->computePHit(zt, z_actual)   + 
            this->zShort * this->computePShort(zt, z_actual) +
            this->zMax   * this->computePMax(zt)             +
            this->zRand  * this->computePrand(zt);

        q *= p;
        angle =+ map->getAngleIcr();
    }
    this->weight = q;
}

double inline Particle::computePHit(double zt, double zt_actual) {
    return erf((zt_actual - zt) / (this->sigmaHit * sqrt(2))) * 
            Math::model_normal_distribution(zt, zt_actual, this->sigmaHit);
}

double inline Particle::computePShort(double zt, double zt_actual){
    return (1 / (1 - exp(-this->lambdaShort * zt_actual))) * 
            this->lambdaShort * exp(-this->lambdaShort * zt);
} 

double inline Particle::computePMax(double zt){
    return (zt >= map->getMaxRange()) ? 1.f: 0.f;
}
double inline Particle::computePrand(double zt) {
    return (zt > 0.f && zt < map->getMaxRange()) ? (1 / map->getMaxRange()) : 0.f;
}
