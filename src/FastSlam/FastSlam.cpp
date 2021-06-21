#include <FastSlam/FastSlam.h>

FastSlam::FastSlam() 
{
  
}

FastSlam::~FastSlam() 
{

}

void FastSlam::Measurement_model(const ParticlePoint& p, int LandMarkID, Eigen::Vector2d& h, Eigen::Matrix2d& H) 
{
    Point landmark_point = p.landmark_list[LandMarkID].Nearest_point;//機器人與地標最近的點
    Point particle_point = Point(p.pos.pose.x,p.pos.pose.y);//粒子點位置

    //use the current state of particle to predict measuremen
    float delta_x = landmark_point.x - particle_point.x;
    float delta_y = landmark_point.y - particle_point.y;
    double expect_distance   = sqrt(delta_x * delta_x + delta_y * delta_y);
    double expect_angle = normalize_angle(p.landmark_list[LandMarkID].Line_theta - p.pos.angle);
    // ROS_INFO("delta_x = %f",delta_x);
    // ROS_INFO("delta_y = %f",delta_y);
    // ROS_INFO("expect_distance = %f",expect_distance);
    // ROS_INFO("expect_angle = %f",expect_angle);
    h << expect_distance, expect_angle;
    // h << 0, 0;

    //compute the Jacobian H of the measurement function h wrt to the landmark location
    H = Eigen::Matrix2d::Zero(2, 2);
    H(0, 0) = delta_x  / expect_distance;
    H(0, 1) = delta_y  / expect_distance;
    H(1, 0) = -delta_y / (expect_distance * expect_distance);
    H(1, 1) = delta_x / (expect_distance * expect_distance);
}

