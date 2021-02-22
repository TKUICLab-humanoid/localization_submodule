#include <FastSlam/FastSlam.h>

FastSlam::FastSlam() 
{
  
}

FastSlam::~FastSlam() 
{

}

void FastSlam::Initialize(ParticlePoint& p,unsigned int landmark_size, int _N = 50) 
{
//   noises << 0.005, 0.01, 0.005;
  Q_ << 0.1, 0, 0, 0.1;
  N = _N;

    for (int i = 0; i < N; i++) 
    {
        p.likehood = 1.0 / N;
        p.pos.pose = Point(0,0);
        p.pos.theta = 0;
        p.landmark_list.resize(landmark_size);
        for (int j = 0; j < landmark_size; j++) 
        {
            p.landmark_list[j].mu << 0,0;
            p.landmark_list[j].sigma << 0, 0, 0, 0;
        }
    }
}

void FastSlam::Measurement_model(const ParticlePoint& p, int LandMarkID, Eigen::Vector2d& h, Eigen::MatrixXd& H) 
{
    Point landmark_point = p.landmark_list[LandMarkID].Nearest_point;//機器人與地標最近的點
    Point particle_point = Point(p.postion.x,p.postion.y);//粒子點位置
    //use the current state of particle to predict measuremen
    float delta_x = landmark_point.x-particle_point.x;
    float delta_y = landmark_point.y-particle_point.y;
    double expect_distance   = sqrt(delta_x * delta_x + delta_y * delta_y);
    double expect_angle = normalize_angle(atan2(delta_y, delta_x) - p.angle);

    h << expect_distance, expect_angle;

    //compute the Jacobian H of the measurement function h wrt to the landmark location
    H = Eigen::MatrixXd::Zero(2, 2);
    H(0, 0) = delta_x  / expect_distance;
    H(0, 1) = delta_y  / expect_distance;
    H(1, 0) = -delta_y / (expect_distance * expect_distance);
    H(1, 1) = delta_x / (expect_distance * expect_distance);
}
