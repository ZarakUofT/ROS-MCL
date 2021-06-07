#include "mcl.h"

MCL::MCL(uint num_particles, std::shared_ptr<OdomData> odom_data, 
        std::shared_ptr<LidarData> lidar_data, std::shared_ptr<Map> _map) 
    : numParticles(num_particles), particles({}), odomData(odom_data), lidarData(lidar_data), map(_map){
    this->initParticles();
}

MCL::~MCL() {
    
}


void MCL::initParticles() {
    uint map_size = this->map->getMapSize();
    uint pos_x, pos_y;
    std::vector<uint> rand_nums;

    Particle::setOdomModel(ALPHA1, ALPHA2, ALPHA3, ALPHA4);
    Particle::setBeamModel(ZHIT, ZMAX, ZSHORT, ZRAND, SIGMAHIT, LAMBDASHORT, CHIOUTLIER);
    Particle::setStatics(odomData, lidarData, map);

    Math::random_integers_in_range(static_cast<uint>(0), map_size, this->numParticles, rand_nums);
    for (uint i = 0; i < this->numParticles; i++){
        this->map->ARR_1D_to_2D(rand_nums[i], pos_x, pos_y);
        auto pose = std::make_shared<Pose>(this->map->getPose(pos_x, pos_y));
        this->particles.push_back(Particle(pose, INITIAL_PARTICLE_WEIGHT, pos_x, pos_y));
    }
}
