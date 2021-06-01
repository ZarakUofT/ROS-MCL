#include "particle.h"
#include "map.h"

//Global Variables
const double LIDAR_MAX_RANGE = 3.5; // in meters
const double ANGLE_INCREMENT = 0.017501922324299812;
const int MAP_RENDER_CYCLE = 5;

//Temp Global Variable
std::shared_ptr<OdomData> odom_data = std::make_shared<OdomData>();

//Callbacks
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

}

void odomCallback(const nav_msgs::Odometry::ConstPtr & msg){
    odom_data->odomCallback(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MCL");
    ros::NodeHandle nh;
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);

    ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    auto start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    std::shared_ptr<Map> map = std::make_shared<Map>(LIDAR_MAX_RANGE, ANGLE_INCREMENT);
    std::string pathToFile = "/home/zarak/mcl/src/mcl/src";

    map->loadMapFromFile(pathToFile, "grid_map.csv", true);
    map->update_image();
    map->saveAngularRanges(pathToFile + "/angular_ranges.txt", 268, 316);

    while(ros::ok()) {
        ros::spinOnce();
        // secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();

        if (secondsElapsed > MAP_RENDER_CYCLE){
            // Render Particle filter at some point
            start = std::chrono::system_clock::now();
        }
        loop_rate.sleep();
    }

    return 0;
}
