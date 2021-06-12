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
        this->particles.push_back(Particle(pose, 1/this->numParticles, pos_x, pos_y));
    }
}

void MCL::update() {
    std::vector<Particle> tempParticles = this->particles;
    
    for (auto& p: tempParticles){
        p.applyOdomMotionModel();
        p.applySensorModel();
    }

    // get weights and normalize
    std::vector<double> weights;
    for (int i = 0; i < this->numParticles; i++){
        weights.push_back(tempParticles[i].getWeight());
    }
    Math::normalize_vec(weights);
    
    this->lowVarianceResampler(tempParticles, weights);
}

void MCL::lowVarianceResampler(std::vector<Particle>& tempParticles, std::vector<double>& weights) {
    std::vector<double> c;
    Math::cumalitive_vector(weights, c);
    double r = Math::random_double(0.f, 1 / this->numParticles);
    double u = 0.f;

    uint i = 1;

    // clear the particles vector since it will be repopulated
    this->particles.clear();
    this->particles.reserve(this->numParticles);

    for (size_t m = 1; m <= this->numParticles; m++){
        u = r + (m - 1) * (1/this->numParticles);

        while(u > c[i]){
            i++;
        }
        this->particles.push_back(tempParticles[i]);
    }
}
