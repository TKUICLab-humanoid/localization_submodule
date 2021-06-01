#include "IMCL/IMCL.h"

void IMonteCarlo::Prediction(vector<int> u)
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

void IMonteCarlo::KLD_Sampling()
{
    ROS_INFO("KLD_Sampling");
    int current_num = 0;
    int particlepoint_num_finish = min_particlepoint_num + 10;
    non_empty_bin.clear();
    //ROS_INFO("particle number is %d !!!!!!!!!!!!!!!!!!!!!!!!",particlepoint_num);

    while(current_num < particlepoint_num && current_num < particlepoint_num_finish)
    {
        Pointdata current_point;
        current_point.pose.x = particlepoint[current_num].postion.x;
        current_point.pose.y = particlepoint[current_num].postion.y;
        current_point.angle = int(particlepoint[current_num].angle);

        if(non_empty_bin.size() != 0)
        {
            for(int i = 0; i < non_empty_bin.size(); i++)
            {
                int dis_x = abs(current_point.pose.x - non_empty_bin[i].pose.x);
                int dis_y = abs(current_point.pose.y - non_empty_bin[i].pose.y);
                int dis = sqrt(pow(dis_x,2) + pow(dis_y,2));
                //ROS_INFO("dis = %d",dis);
                // ROS_INFO("current_point.pose.x = %d",current_point.pose.x);
                // ROS_INFO("current_point.pose.y = %d",current_point.pose.y);
                // ROS_INFO("particlepoint[current_num].postion.x = %d",particlepoint[current_num].postion.x);
                // ROS_INFO("particlepoint[current_num].postion.y = %d",particlepoint[current_num].postion.y);
                if(dis > 3)
                {
                    non_empty_bin.push_back(current_point);
                    if(current_num >= min_particlepoint_num)
                    {
                        //ROS_INFO("non_empty_bin.size = %d",non_empty_bin.size());
                        float kld_equation_1 = float((non_empty_bin.size() - 1)) / (2.0 * kld_err);
                        //ROS_INFO("kld_equation_1 is %f !!!!!!!!!!!!!!!!!!!!!!!!",kld_equation_1);
                        float kld_equation_2 = 1.0 - 2.0 / (9.0 * float((non_empty_bin.size() - 1)));
                        //ROS_INFO("kld_equation_2 is %f !!!!!!!!!!!!!!!!!!!!!!!!",kld_equation_2);
                        float kld_equation_3 = sqrt(2.0 / (9.0 * (non_empty_bin.size() - 1))) * kld_z;
                        //ROS_INFO("kld_equation_3 is %f !!!!!!!!!!!!!!!!!!!!!!!!",kld_equation_3);
                        particlepoint_num_finish = int(kld_equation_1 * pow((kld_equation_2 + kld_equation_3),3));  
                    }    
                    break;
                }
            }
        }
        else
        {
            non_empty_bin.push_back(current_point);
        }
        current_num++;
        //ROS_INFO("particle number is %d !!!!!!!!!!!!!!!!!!!!!!!!",particlepoint_num);
    }
    non_empty_bin.clear();
    particlepoint_num = particlepoint_num_finish;
    if(particlepoint_num < min_particlepoint_num)
    {
        particlepoint_num = min_particlepoint_num;
    }
    else if(particlepoint_num > PARTICLNUM)
    {
        particlepoint_num = PARTICLNUM;
    }
    excellent_particle_num = particlepoint_num * 0.1;
    if(excellent_particle_num > EXCELLENTPARTICLNUM)
    {
        excellent_particle_num = EXCELLENTPARTICLNUM;
    }
    ROS_INFO("particle number is %d !!!!!!!!!!!!!!!!!!!!!!!!",particlepoint_num);
    ROS_INFO("KLD_Sampling end!!");
}

int IMonteCarlo::TournamentResample(int excellent_particle_num)
{
    ROS_INFO("TournamentResample");
    int tournament_particle_num[excellent_particle_num] = {0};
    int best_particle_num = 0;
    float best_weight_value = 0.0;

    for(int i = 0; i < excellent_particle_num; ++i)
    {
        tournament_particle_num[i] = rand()%particlepoint_num;
        for(int j = 0; j < i; j++)
        {
            if(tournament_particle_num[i] == tournament_particle_num[j])
            {
                tournament_particle_num[i] = rand()%particlepoint_num;
                j = -1;
            }
        }
        if(particlepoint[tournament_particle_num[i]].weight > best_weight_value)
        {
            best_weight_value = particlepoint[tournament_particle_num[i]].weight;
            best_particle_num = tournament_particle_num[i];
        }
    }
    
    return best_particle_num;
}

void IMonteCarlo::resample()
{
    weight_slow += alpha_slow * (totalweight/(float)particlepoint_num - weight_slow);
    weight_fast += alpha_fast * (totalweight/(float)particlepoint_num - weight_fast);
    std::random_device rd, x_rd, y_rd;                                          //產生亂數種子                    //Produce the random seed
    std::mt19937 generator(rd());                                               //使用梅森旋轉演算法產生亂數        //Use Mersenne Twister to produce the random
    std::mt19937 x_generator(x_rd());
    std::mt19937 y_generator(y_rd());
    std::uniform_real_distribution<float> add_random_distribution(0.0, 1.0);    //設定機率分佈範圍（連續型均勻分佈） //Set the random distribution range(continuous)
    std::uniform_real_distribution<float> x_random_distribution(550, MAP_LENGTH - 105), y_random_distribution(10, MAP_WIDTH - 10);
    float reset_random_threshold = std::max(0.0, 1.0 - (weight_fast / weight_slow));
    
    for(int i = 0; i < particlepoint_num; ++i)
    {
        double random = ((double)rand() / (RAND_MAX));
        ParticlePoint current_particle;
        if(random <= reset_random_threshold)
        {
            current_particle.postion.x = x_random_distribution(x_generator);
            current_particle.postion.y = y_random_distribution(y_generator);
            current_particle.angle = normalize_angle(Robot_Position.angle + rand()%rand_angle - rand_angle_init);
            tmp.push_back(current_particle);
        }
        else
        {
            int best_particle_num = TournamentResample(excellent_particle_num);
            current_particle.angle = normalize_angle(Robot_Position.angle + rand()%rand_angle - rand_angle_init);
            current_particle.postion.x = particlepoint[best_particle_num].postion.x + (rand() % 11 - 5);
            current_particle.postion.y = particlepoint[best_particle_num].postion.y + (rand() % 11 - 5);
            tmp.push_back(current_particle);
        }
    }
    particlepoint.clear();
    particlepoint = tmp;
    
}
