#include "IMCL/IMCL.h"

void IMonteCarlo::Predict(vector<int> u)
{
    ROS_INFO("StatePredict");
    vector<ParticlePoint> tmp;
    //-------------add motion error coefficients---------------
    for(int i = 0; i < particlepoint_num; ++i)
    {
        ParticlePoint current_particle;
        Movement(u[0],u[1],u[2],u[3],u[4]);
        current_particle.pos.pose.x = posx;
        current_particle.pos.pose.y = posy;
        current_particle.pos.angle = rotation;
        tmp.push_back(current_particle);
    }
    particlepoint.clear();
    particlepoint = tmp;
    ROS_INFO("particlepoint size = %d",particlepoint.size());
}

void IMonteCarlo::Update()
{
    use_feature_point = false;
    use_lineinformation = true;
    totalweight = calcWeight(&feature_point_observation_data[0], &Line_observation_data[0]);
    
}

void IMonteCarlo::resample()
{
    weight_slow += alpha_slow * (totalweight/(float)particlepoint_num - weight_slow);
    weight_fast += alpha_fast * (totalweight/(float)particlepoint_num - weight_fast);
    float reset_random_threshold = std::max(0.0, 1.0 - (weight_fast / weight_slow));
    for(int i = 0; i < particlepoint_num; ++i)
    {
        double random = ((double)rand() / (RAND_MAX));
        ParticlePoint current_particle;
        if(random <= reset_random_threshold)
        {
            
        }
        else
        {

        }
    }

}
