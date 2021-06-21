#ifndef __MCL_H__
#define __MCL_H__

#include "particle.h"

const double ALPHA1      =  0.2;
const double ALPHA2      =  0.2;
const double ALPHA3      =  0.2;
const double ALPHA4      =  0.2;
const double ZHIT        =  0.80;
const double ZMAX        =  0.05;
const double ZSHORT      =  0.10;
const double ZRAND       =  0.05;
const double SIGMAHIT    =  0.2;
const double LAMBDASHORT =  0.1;
const double CHIOUTLIER  =  0.1;
const double MIN_DIST    = 0.01;

class MCL{
private:
    uint numParticles;
    std::vector<Particle> particles;

    std::shared_ptr<OdomData> odomData;
    std::shared_ptr<LidarData> lidarData;
    std::shared_ptr<Map> map;

    //figure stuff
    figure_handle figure;
    axes_handle axes;

    // Held for plotting for graph on a different thread
    std::future<bool> futurePlotThread;
    bool stillPlotting;
    bool threadCreatedForPlotting;

    // private funcs
    void computeParticleRots(double& delta_rot1, double& delta_trans, double& delta_rot2, 
                            double& delta_rot1_noise, double& delta_rot2_noise);
public:
    MCL(uint num_particles, std::shared_ptr<OdomData> odom_data, 
        std::shared_ptr<LidarData> lidar_data, std::shared_ptr<Map> _map);
    virtual ~MCL();

    void initParticles();
    void initFigure();

    void update();

    void lowVarianceResampler(std::vector<Particle>& tempParticles, std::vector<double>& weights);

    void update_graph();
};

bool draw(std::shared_ptr<std::vector<uint>> x, std::shared_ptr<std::vector<uint>> y, axes_handle& axes,
            std::shared_ptr<std::vector<uint>> size, std::shared_ptr<std::vector<matplot::color>> color);

#endif // __MCL_H__