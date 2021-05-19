#include "particle.h"

Particle::Particle(Pose _pose, Pose prev_pose) 
    :pose(_pose), prevPose(prev_pose){
}

Particle::~Particle() 
{
    
}

static void Particle::initParticle(double a1, double a2, double a3, double a4) 
{
    this->alpha1 = a1;
    this->alpha2 = a2;
    this->alpha3 = a3;
    this->alpha4 = a4;
}

void Particle::update() 
{
    
}

void Particle::applyOdomMotionModel(Pose* newOdom) 
{
    if (!newOdom)
        return;
    
    double delta_rot1, delta_trans, delta_rot2;
    double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
    double delta_rot1_noise, delta_rot2_noise;

    delta_rot1 = Math::angle_diff(atan2(newOdom->y - this->prevOdom->y, newOdom->x - this->prevOdom->x), this->prevOdom->yaw);
    delta_trans = sqrt(pow((this->prevOdom->x - newOdom->x), 2) + pow((this->prevOdom->y - newOdom->y), 2));
    delta_rot2 = Math::angle_diff(newOdom->yaw - this->prevOdom->yaw, delta_rot1);
}