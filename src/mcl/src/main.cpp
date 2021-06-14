#include "particle.h"
#include "map.h"
#include "mcl.h"

//Global Variables
const double LIDAR_MAX_RANGE = 3.5; // in meters
const double ANGLE_INCREMENT = 0.017501922324299812;
const int MAP_RENDER_CYCLE = 5;

const uint NUM_PARTICLES = 20;

int main(int argc, char **argv)
{
    // seed random number
    srand (static_cast <unsigned> (time(0)));

    ros::init(argc, argv, "MCL");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    std::shared_ptr<OdomData> odom_data = std::make_shared<OdomData>(nh);
    std::shared_ptr<LidarData> lidar_data = std::make_shared<LidarData>(nh);

    // geometry_msgs::Twist vel;

    auto start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
    
    std::shared_ptr<Map> map = std::make_shared<Map>(90, LIDAR_MAX_RANGE, ANGLE_INCREMENT, 0, 0);
    std::string pathToFile = "/home/zarak/mcl/src/mcl/src";
    map->loadMapFromFile(pathToFile, "grid_map.csv", true);
    // map->update_image();

    std::shared_ptr<MCL> mcl = std::make_shared<MCL>(NUM_PARTICLES, odom_data, lidar_data, map);
    while(ros::ok()) {
        ros::spinOnce();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        mcl->update();
        if (secondsElapsed > MAP_RENDER_CYCLE){
            // Render Particle filter at some point
            mcl->draw();
            start = std::chrono::system_clock::now();
        }
        loop_rate.sleep();
    }

    return 0;
}
