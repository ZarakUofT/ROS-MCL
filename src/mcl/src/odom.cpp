#include "odom.h"

OdomData::OdomData(ros::NodeHandle& n)
    : nh(n)    
{
    this->delta = std::make_shared<odom_t> (0.0, 0.0, 0.0);
    this->currOdom = std::make_shared<odom_t> (0.0, 0.0, 0.0);

    this->odomSuscriber = this->nh.subscribe("odom", 10, &OdomData::callback, this);
}

OdomData::OdomData(ros::NodeHandle& n, std::shared_ptr<odom_t> curr_odom) 
    : nh(n), currOdom(curr_odom)
{
    this->delta = std::make_shared<odom_t> (0.0, 0.0, 0.0);

    this->odomSuscriber = this->nh.subscribe("odom", 10, &OdomData::callback, this);
}

OdomData::OdomData(ros::NodeHandle& n,
        std::shared_ptr<odom_t> delt, std::shared_ptr<odom_t> curr_odom) 
    : nh(n), delta(delt), currOdom(curr_odom)
{
    this->odomSuscriber = this->nh.subscribe("odom", 10, &OdomData::callback, this);
}

OdomData::~OdomData() {}

void OdomData::callback(const nav_msgs::Odometry::ConstPtr & msg) 
{
    double x, y, yaw;
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    yaw = Math::angle_proper_range(yaw);

    // compute delta
    this->delta->x = x - this->currOdom->x;
    this->delta->y = y - this->currOdom->y;
    this->delta->yaw = Math::angle_diff(yaw, this->currOdom->yaw);

    // set current odom
    this->currOdom->x = x;
    this->currOdom->y = y;
    this->currOdom->yaw = yaw;
}
