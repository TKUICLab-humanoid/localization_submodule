#include <FastSlam/FastSlam.h>

FastSlam::FastSlam() 
{
  
}

FastSlam::~FastSlam() 
{

}

void FastSlam::Measurement_model(const ParticlePoint& p, int LandMarkID, Eigen::Vector2d& h, Eigen::MatrixXd& H) 
{
    Point landmark_point = p.landmark_list[LandMarkID].Nearest_point;//機器人與地標最近的點
    
    Point particle_point = Point(p.pos.pose.x,p.pos.pose.y);//粒子點位置

    // ROS_INFO("landmark_point %d %d",landmark_point.x,landmark_point.y);
    // ROS_INFO("particle_point %d %d",particle_point.x,landmark_point.y);
    //use the current state of particle to predict measuremen
    float delta_x = abs(landmark_point.x - particle_point.x);
    float delta_y = abs(landmark_point.y - particle_point.y);
    double expect_distance   = sqrt(delta_x * delta_x + delta_y * delta_y);
    double expect_angle = normalize_angle(atan2(delta_y, delta_x) - p.pos.angle);
    // ROS_INFO("delta_x = %f",delta_x);
    // ROS_INFO("delta_y = %f",delta_y);
    // ROS_INFO("expect_distance = %f",expect_distance);
    // ROS_INFO("expect_angle = %f",expect_angle);
    h << expect_distance, expect_angle;

    //compute the Jacobian H of the measurement function h wrt to the landmark location
    H = Eigen::MatrixXd::Zero(2, 2);
    H(0, 0) = delta_x  / expect_distance;
    H(0, 1) = delta_y  / expect_distance;
    H(1, 0) = -delta_y / (expect_distance * expect_distance);
    H(1, 1) = delta_x / (expect_distance * expect_distance);

}
