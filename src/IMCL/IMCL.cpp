#include "IMCL/IMCL.h"

IMonteCarlo::IMonteCarlo()
{
    noises << 0.005, 0.01, 0.005;
    Q_ << 0.1, 0, 0, 0.1;

    
    rand_angle_init = 5;
    particlepoint_num = PARTICLNUM;
    excellent_particle_num = EXCELLENTPARTICLNUM;
    Robot_Position.postion.x = -1; //830  700
    Robot_Position.postion.y = -1; //640  370
    Robot_Position.angle = -1.0;

    step_count = 0;
    //////////////////KLD//////////////////
    min_particlepoint_num = 50;
    kld_err = 0.45;             //defalut 0.05
    kld_z = 0.99;               //defalut 0.99
    //////////////////KLD//////////////////

    //////////////////Augmented_MCL//////////////////
    weight_avg = 0.0;       //the average of the weight of particlepoint
    weight_slow = 0.0;      //the long-term weight of particlepoint
    weight_fast = 0.0;      //the short-term weight of particlepoint
    alpha_slow = 0.003;     //0.062;
    alpha_fast = 0.1;       //0.1;
    //////////////////Augmented_MCL//////////////////
    total_weight = 0;
    localization_flag = true;
    find_best_flag = true;
    use_feature_point = false;
    use_lineinformation = true;
    AngleLUT();

}
IMonteCarlo::~IMonteCarlo()
{
    Angle_sin.clear();
    Angle_cos.clear();
}

/**Initialize the parameters**/
void IMonteCarlo::Initialize(unsigned int landmark_size)
{
    ROS_INFO("ParticlePointinit");
    localization_flag = true;
    // noise << 0.005, 0.01, 0.005;
    
    for (auto& p : particles)
    {
        p.pos.pose = Point(0,0);
        p.pos.angle = 0.0;
        p.factors = factors;
        p.wfactors = wfactor;
        p.fitness_value = 0.0;
        p.likehood = 1.0 / particlepointnum;
        p.landmark_list.resize(landmark_size);
        for (int j = 0; j < landmark_size; j++) 
        {
            p.landmark_list[j].mu << 0,0;
            p.landmark_list[j].sigma << 0, 0, 0, 0;
        }
        particlepoint.push_back(p);
    }
}

void IMonteCarlo::Prediction(const movement_data& u)
{
    ROS_INFO("StatePredict");
    // vector<ParticlePoint> tmp;
    //-------------add motion error coefficients---------------
    for(auto& p : particles)
    {
        // ParticlePoint current_particle;
        Movement(u.straight,u.drift,u.rotational,u.moving,u.dt);
        p.pos.pose.x = posx;
        p.pos.pose.y = posy;
        p.pos.angle = rotation;
        // tmp.push_back(p);
    }
    // particlepoint.clear();
    // particlepoint = tmp;
    ROS_INFO("particlepoint size = %d",particlepoint.size());
}

void IMonteCarlo::Update(const vector<observation_data>& observations)
{
    use_feature_point = false;
    use_lineinformation = true;
    int weightsum = 0;
    for(auto& p : particles)
    {
        for (const auto& z : observations)
        {
            weightsum += calcWeight(z.featurepoint_scan_line, z.landmark_list);

        }
        totalweight += weightsum;
    }
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
            find_best_flag = true;
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

void IMonteCarlo::NoLookField(const movement_data& u)
{
    ROS_INFO("NoLookField");

    for(int i = 0; i < particlepoint_num; ++i)
    {
        posx = particlepoint[i].postion.x;
        posy = particlepoint[i].postion.y;
        rotation = particlepoint[i].angle;
        Movement(u.straight,u.drift,u.rotational,u.moving,u.dt);          
        particlepoint[i].postion.x = posx;
        particlepoint[i].postion.y = posy;
        particlepoint[i].angle = rotation;
        ROS_INFO("posx = %d,posy = %d,rotation = %d",posx,posy,rotation);
    }
    posx = Robot_Position.postion.x;
    posy = Robot_Position.postion.y;
    rotation = Robot_Position.angle;
    Movement(u.straight,u.drift,u.rotational,u.moving,u.dt); 
    Robot_Position.postion.x = posx;
    Robot_Position.postion.y = posy;
    Robot_Position.angle = rotation;
    ROS_INFO("posx = %d,posy = %d,rotation = %d",posx,posy,rotation);

    // float move_x = 0.0;
    // float move_y = 0.0;
    // float y_dir_angle = 0.0;
    // for(int i = 0; i < particlepoint_num; ++i)
    // { 
    //     particlepoint[i].angle = normalize_angle(Robot_Position.angle);
    //     if(continuous_y > 0)
    //     {
    //         float y_dir_angle = normalize_angle(particlepoint[i].angle + 90.0);
    //     }
    //     else
    //     {
    //         float y_dir_angle = normalize_angle(particlepoint[i].angle - 90.0);
    //     }
    //     move_x = cos(particlepoint[i].angle * DEG2RAD) * continuous_x / 1000 + cos(y_dir_angle * DEG2RAD) * continuous_y / 1000;
    //     move_y = -(sin(particlepoint[i].angle * DEG2RAD) * continuous_x / 1000 + sin(y_dir_angle * DEG2RAD) * continuous_y / 1000);
    //     move_x = 6.5 * move_x;
    //     move_y = 6.5 * move_y;
    //     if(step_count == 0)
    //     {
    //         particlepoint[i].postion.x = particlepoint[i].postion.x + (int)(move_x);
    //         particlepoint[i].postion.y = particlepoint[i].postion.y + (int)(move_y);
    //         //ROS_INFO("add the step");
    //     }
    //     else
    //     {
    //         particlepoint[i].postion.x = particlepoint[i].postion.x + (int)(move_x) * step_count;
    //         particlepoint[i].postion.y = particlepoint[i].postion.y + (int)(move_y) * step_count;
    //         //ROS_INFO("add the step");
    //         //step_count = 0;
    //     }
    // }
    // Robot_Position.angle = normalize_angle(Robot_Position.angle);
    // if(continuous_y > 0)
    // {
    //     float y_dir_angle = normalize_angle(Robot_Position.angle + 90.0);
    // }
    // else
    // {
    //     float y_dir_angle = normalize_angle(Robot_Position.angle - 90.0);
    // }
    // move_x = cos(Robot_Position.angle * DEG2RAD) * continuous_x / 1000 + cos(y_dir_angle * DEG2RAD) * continuous_y / 1000;
    // move_y = -(sin(Robot_Position.angle * DEG2RAD) * continuous_x / 1000 + sin(y_dir_angle * DEG2RAD) * continuous_y / 1000);
    // move_x = 6.5 * move_x;
    // move_y = 6.5 * move_y;
    // if(step_count == 0)
    // {
    //     Robot_Position.postion.x = Robot_Position.postion.x + (int)(move_x);
    //     Robot_Position.postion.y = Robot_Position.postion.y + (int)(move_y);
    //     //ROS_INFO("add the step");
    // }
    // else
    // {
    //     Robot_Position.postion.x = Robot_Position.postion.x + (int)(move_x) * step_count;
    //     Robot_Position.postion.y = Robot_Position.postion.y + (int)(move_y) * step_count;
    //     //ROS_INFO("add the step");
    //     //step_count = 0;
    // }
    localization_flag = false;
}


