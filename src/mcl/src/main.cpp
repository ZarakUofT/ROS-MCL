#include "particle.h"

#define RAD2DEG(rad)((rad) * 180 / M_PI)
#define DEG2RAD(deg)((deg) * M_PI/180)

//Global Variables
const float LIDAR_MAX_RANGE = 3.5; // in meters

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

    while(ros::ok()) {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
