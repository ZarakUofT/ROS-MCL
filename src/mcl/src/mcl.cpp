#include "mcl.h"

MCL::MCL(uint num_particles, std::shared_ptr<OdomData> odom_data, 
        std::shared_ptr<LidarData> lidar_data, std::shared_ptr<Map> _map) 
    : numParticles(num_particles), particles({}), odomData(odom_data), lidarData(lidar_data), map(_map),
        stillPlotting(false), threadCreatedForPlotting(false)
{
    this->initFigure();
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
        this->particles.push_back(Particle(pose, 1 / static_cast<double>(this->numParticles), pos_x, pos_y));
    }
}

void MCL::initFigure() {
    this->figure = matplot::figure();
    this->figure->ioff();
    this->axes = matplot::axes(this->figure, true);
    this->axes->x_axis().limits({0.0, static_cast<double>(this->map->getMapWidth())});
    this->axes->y_axis().limits({0.0, static_cast<double>(this->map->getMapHeight())});
    // this->axes->grid(false);
}

void MCL::computeParticleRots(double& delta_rot1, double& delta_trans, double& delta_rot2, 
                            double& delta_rot1_noise, double& delta_rot2_noise)
{
    if (sqrt(pow(this->odomData->delta->x, 2) + pow(this->odomData->delta->y, 2)) < MIN_DIST) {
        delta_rot1 = 0.f;
    }else{
        delta_rot1 = Math::angle_diff(atan2(this->odomData->delta->y, this->odomData->delta->x), 
                        this->odomData->currOdom->yaw);
    }
    delta_trans = sqrt(pow((this->odomData->delta->x), 2) + pow((this->odomData->delta->y), 2));
    delta_rot2 = Math::angle_diff(this->odomData->delta->yaw, delta_rot1);

    // We want to treat backward and forward motion symmetrically for the
    // noise model to be applied below.
    delta_rot1_noise = std::min(fabs(Math::angle_diff(delta_rot1, 0.0)),
                                fabs(Math::angle_diff(delta_rot1, M_PI)));
    delta_rot2_noise = std::min(fabs(Math::angle_diff(delta_rot2, 0.0)),
                                fabs(Math::angle_diff(delta_rot2, M_PI)));
}

void MCL::update() {
    std::vector<Particle> tempParticles = this->particles;

    double delta_rot1, delta_trans, delta_rot2;
    double delta_rot1_noise, delta_rot2_noise;

    this->computeParticleRots(delta_rot1, delta_trans, delta_rot2, delta_rot1_noise, delta_rot2_noise);
    
    for (auto& p: tempParticles) {
        // std::cout << "x_A: " << odomData->currOdom->x << ", y_A: " << odomData->currOdom->y << 
        //             ", yaw_A: " << odomData->currOdom->yaw << std::endl;
        // std::cout << "x: " << p.getPose()->x << ", y: " << p.getPose()->y << ", yaw: " << p.getPose()->yaw << std::endl;
        p.update(delta_rot1, delta_trans, delta_rot2, delta_rot1_noise, delta_rot2_noise);
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

    double r = Math::random_double(0.f, 1 / static_cast<double>(this->numParticles));
    double u = 0.f;

    uint i = 0;

    // for (auto& w: weights){
    //     std::cout << w << ", ";
    // }
    // std::cout << std::endl;

    // clear the particles vector since it will be repopulated
    this->particles.clear();
    this->particles.reserve(this->numParticles);

    for (size_t m = 1; m <= this->numParticles; m++){
        u = r + (m - 1) * (1 / static_cast<double>(this->numParticles));

        while(u > c[i] && i < tempParticles.size() - 1){
            i++;
        }
        this->particles.push_back(tempParticles.at(i));
    }
}

void MCL::update_graph() {
    if (this->threadCreatedForPlotting)
        this->stillPlotting = this->futurePlotThread.wait_for(std::chrono::seconds(0))!=std::future_status::ready;
    if (this->stillPlotting)
        return;

    std::vector<uint> x;
    std::vector<uint> y;
    std::vector<uint> size;
    std::vector<matplot::color> color;

    for (auto &p:particles){
        std::pair<uint32_t, uint32_t> pos = p.getMapPos();
        x.push_back(pos.first);
        y.push_back(pos.second);
        size.push_back(10.0);
        color.push_back(matplot::color::green);
    }

    auto x_ptr = std::make_shared<std::vector<uint>>(x);
    auto y_ptr = std::make_shared<std::vector<uint>>(y);
    auto size_ptr = std::make_shared<std::vector<uint>>(size);
    auto color_ptr = std::make_shared<std::vector<matplot::color>>(color);

    // Start a new thread
    this->futurePlotThread = std::async(std::launch::async, draw, x_ptr, y_ptr, std::ref(this->axes), size_ptr, color_ptr);
    this->threadCreatedForPlotting = true;
}   

/* 
    Non-class Functions
*/

bool draw(std::shared_ptr<std::vector<uint>> x, std::shared_ptr<std::vector<uint>> y, axes_handle& axes,
            std::shared_ptr<std::vector<uint>> size, std::shared_ptr<std::vector<matplot::color>> color) 
{
    auto m = axes->scatter(*x, *y, *size, *color);
    m->marker_face(true);
    m->marker_style(line_spec::marker_style::diamond);
    axes->draw();

    return true;
}