#include "ParticleFilter/ParticleFilter.h"

ParticleFilter::ParticleFilter()
{
    rand_angle_init = 5;
    particlepoint_num = PARTICLNUM;
    excellent_particle_num = EXCELLENTPARTICLNUM;

    Robot_Position.pos.pose.x = 750; //830  700
    Robot_Position.pos.pose.y = 213; //640  370
    Robot_Position.pos.angle = 0.0;

    continuous_x = 0;
    continuous_y = 0;

    step_count = 0;

    //////////////////W_MCL////////////////
    
    // factors = { 1, 2, 1, 500, 5,
    //             1, 2, 1, 500, 7,
    //             1, 2, 1, 100, 5};
    // factors = { 0, 0, 0, 0,  0,  0, 0, 0, 0,  0,  0, 0, 0, 0,  0};
    // factors = { 1, 2, 1, 0, 10,  1, 2, 1, 0, 20,  1, 2, 1, 0, 10};
    // factors = { 0, 0, 0, 0, 10,  0, 0, 0, 0, 20,  0, 0, 0, 0, 10};
    factors = { -1, 0, 1, 50, 5,  0, 0, 0, 50, 5,  -1, 0, 1, 10, 5};
    
    regions = {Point(0, 1100), Point(0, 800), Point(-180, 180)};
    // posx = init_robot_pos_x;
    // posy = init_robot_pos_y;
    rotation = init_robot_pos_dir;
    total_weight = 0.0;
    R << 0.01, 0, 0, 0.001;
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

    localization_flag = true;
    find_best_flag = true;
    use_feature_point = true;
    use_lineinformation = true;
    SigmaIMU = 5;

    AngleLUT();
}
ParticleFilter::~ParticleFilter()
{
    Angle_sin.clear();
    Angle_cos.clear();
}

/**Initialize the parameters**/
void ParticleFilter::ParticlePointInitialize(unsigned int landmark_size)
{
    ROS_INFO("ParticlePointinit");
    ParticlePoint tmp;
    particlepoint.clear();
    localization_flag = true;
    time_t t;
    int rand_angle = rand_angle_init * 2 + 1;        //粒子隨機角度//大數   rand()%40 - 20 = -20 ~ 20

    srand((unsigned) time(&t));
    // noise << 0.005, 0.01, 0.005;
    for (int i = 0; i < particlepoint_num; i++)
    {
        tmp.pos.angle = normalize_angle(Robot_Position.pos.angle + rand()%rand_angle - rand_angle_init);
        tmp.pos.pose.x = Robot_Position.pos.pose.x + (rand() % 31 - 15);   //-3 ~ 3
        tmp.pos.pose.y = Robot_Position.pos.pose.y + (rand() % 31 - 15);
        tmp.weight = 0.0;
        tmp.wfactors = 0.0;
        tmp.landmark_list.resize(landmark_size);
        for (int j = 0; j < landmark_size; j++) 
        {
            // tmp.landmark_list[j].mu << 0,0;
            tmp.landmark_list[j].sigma << 0, 0, 0, 0;
            tmp.landmark_list[j].obersvated = false;
            tmp.landmark_list[j].Line_theta = 0.0;
            tmp.landmark_list[j].start_point = Point(0,0);
            tmp.landmark_list[j].end_point = Point(0,0);
            tmp.landmark_list[j].Nearest_point = Point(-1,-1);
        }
        particlepoint.push_back(tmp);
        // ROS_INFO("particlepoint pos= %d %d",tmp.pos.pose.x,tmp.pos.pose.y);

    }
    // ROS_INFO("particlepoint = %d",particlepoint.size());
}

void ParticleFilter::StatePredict(const movement_data& u)
{
    ROS_INFO("StatePredict");
    if(excellent_particle_num > particlepoint_num)
    {
        excellent_particle_num = particlepoint_num;
    }
    ///////////////////////////Augmented_MCL ///////////////////////////
    weight_slow += alpha_slow * (totalweights/(float)particlepoint_num - weight_slow);
    weight_fast += alpha_fast * (totalweights/(float)particlepoint_num - weight_fast);
    std::random_device rd, x_rd, y_rd;                                          //產生亂數種子                    //Produce the random seed
    std::mt19937 generator(rd());                                               //使用梅森旋轉演算法產生亂數        //Use Mersenne Twister to produce the random
    std::mt19937 x_generator(x_rd());
    std::mt19937 y_generator(y_rd());
    std::uniform_real_distribution<float> add_random_distribution(0.0, 1.0);    //設定機率分佈範圍（連續型均勻分佈） //Set the random distribution range(continuous)
    std::uniform_real_distribution<float> x_random_distribution(550, MAP_LENGTH - 105), y_random_distribution(10, MAP_WIDTH - 10);
    float reset_random_threshold = std::max(0.0, 1.0 - (weight_fast / weight_slow));
    int rand_particle_cnt = 0;
    int rand_angle = rand_angle_init * 2 + 1;        //粒子隨機角度//大數   rand()%40 - 20 = -20 ~ 20
    vector<ParticlePoint> tmp;

    for(int i = 0; i < particlepoint_num; ++i)
    {
        double random = ((double)rand() / (RAND_MAX));
        ParticlePoint current_particle;
        if(random < reset_random_threshold)
        {
            // ROS_INFO("Kidnapped");
            find_best_flag = true;
            current_particle.pos.pose.x = x_random_distribution(x_generator);
            current_particle.pos.pose.y = y_random_distribution(y_generator);
            current_particle.pos.angle = normalize_angle(Robot_Position.pos.angle + rand()%rand_angle - rand_angle_init);
            current_particle.wfactors = 0.0;
            current_particle.weight = 0.0;
            current_particle.landmark_list.resize(field_list.size());
            // current_particle.weight = 1.0/(float)particlepoint_num;
            for(int j = 0; j< field_list.size(); j++)
            {
                current_particle.landmark_list[j].obersvated = false;
                current_particle.landmark_list[j].sigma << 0,0,0,0;
                current_particle.landmark_list[j].Line_theta = 0.0;
                current_particle.landmark_list[j].start_point = Point(0,0);
                current_particle.landmark_list[j].end_point = Point(0,0);
                current_particle.landmark_list[j].Nearest_point = Point(-1,-1);
                // current_particle.landmark_list[j].realdistance = 0.0;
            }
            tmp.push_back(current_particle);
        }
        else
        {
            int best_particle_num = TournamentResample(excellent_particle_num);
            current_particle.pos.angle = normalize_angle(Robot_Position.pos.angle);
            current_particle.landmark_list = particlepoint[best_particle_num].landmark_list;
            current_particle.wfactors = particlepoint[best_particle_num].wfactors;
            current_particle.weight = 0.0;
            Movement(particlepoint[best_particle_num],u.straight,u.drift,u.rotational,u.moving,u.dt);
            if(Step_flag)
            {   
                current_particle.pos.pose.x = particlepoint[best_particle_num].pos.pose.x + (rand() % 11 - 5);
                current_particle.pos.pose.y = particlepoint[best_particle_num].pos.pose.y + (rand() % 11 - 5);
            }else{
                current_particle.pos.pose.x = particlepoint[best_particle_num].pos.pose.x;
                current_particle.pos.pose.y = particlepoint[best_particle_num].pos.pose.y;
            }
            tmp.push_back(current_particle);
        }
    }
    
    particlepoint.clear();
    particlepoint = tmp;
    Step_flag = false;
    // ROS_INFO("particlepoint size = %d",particlepoint.size());
}

void ParticleFilter::Movement(ParticlePoint &p,float straight, float drift, float rotational, float moving, float dt)
{
    // ROS_INFO("Movement");    
    if(int(moving) == 1)
    {
        Motion(p,straight, drift, rotational, dt);
        // ROS_INFO("1");
    }else if(int(moving) == 2)
    {
        GetUpBackUp();
        // ROS_INFO("2");
    }else if(int(moving) == 3){
        GetUpFrontUp();
        // ROS_INFO("3");
    }else{
        Motion(p,0., 0., 0., dt);
    }
}

void ParticleFilter::Motion(ParticlePoint &p,float straight, float drift, float rotational, float dt)
{
    // ROS_INFO("Motion"); 
    // ROS_INFO("straight = %f ,drift = %f ,rotational = %f,dt = %f",straight,drift,rotational,dt); 
    int Forward = int(  straight +
                        gauss(factors[0] * straight) +
                        gauss(factors[1] * drift) +
                        gauss(factors[2] * rotational) +
                        gauss(factors[3] * p.wfactors) +
                        gauss(factors[4]));
    
    int Side  = int(    drift +
                        gauss(factors[5] * straight) +
                        gauss(factors[6] * drift) +
                        gauss(factors[7] * rotational) +
                        gauss(factors[8] * p.wfactors) +
                        gauss(factors[9]));

    float Omega =       rotational +
                        gauss(factors[10] * straight) +
                        gauss(factors[11] * drift) +
                        gauss(factors[12] * rotational) +
                        gauss(factors[13] * p.wfactors) +
                        gauss(factors[14]); 

    
    // ROS_INFO("Forward = %d ,Side = %d ,Omega = %f ",Forward,Side,Omega);
    int posx = p.pos.pose.x;
    int posy = p.pos.pose.y;
    rotation = p.pos.angle;
    Omega = Omega * DEG2RAD;
    float Theta =  rotation * DEG2RAD;
    int x = 0;
    int y = 0;
    float Direction = 0.0;
    float Dir2 = 0.0;
    
    // ROS_INFO("posx = %d posy = %d rotation = %f",posx,posy,rotation );

    if(Omega == 0.0)
    {
        Direction = Theta;
        x = int(round(float(posx) +
                (float(Forward) * cos(Direction) * dt) +
                (float(Side) * sin(Direction) * dt)));
        y = int(round(float(posy) -
                (float(Forward) * sin(Direction) * dt) +
                (float(Side) * cos(Direction) * dt)));
        // ROS_INFO("dt = %f, Direction = %f, sin(Direction) = %f, cos(Direction) = %f",dt, Direction,sin(Direction),cos(Direction));
        // ROS_INFO("x = %d posx = %d ,float(Forward) * cos(Direction) * dt = %f, float(Side) * sin(Direction) * dt = %f",x,posx,float(Forward) * cos(Direction) * dt,float(Side) * sin(Direction) * dt);
        // ROS_INFO("y = %d posy = %d ,float(Forward) * sin(Direction) * dt = %f, float(Side) * cos(Direction) * dt = %f",y,posy,float(Forward) * sin(Direction) * dt,float(Side) * cos(Direction) * dt);
    }else{
        Direction = Theta + Omega * dt;
        Dir2 = -Theta + Omega * dt;
        x = int(round(float(posx) +
            (float(Forward) / Omega * (sin(Direction) - sin(Theta))) -
            (float(Side) / Omega * (cos(-Theta) - cos(Dir2)))));
        y = int(round(float(posy) -
            (float(Forward) / Omega * (cos(Theta) - cos(Direction))) -
            (float(Side) / Omega * (sin(-Theta) - sin(Dir2)))));
        // ROS_INFO("dt = %f, Direction = %f, sin(Direction) = %f, cos(Direction) = %f",dt, Direction,sin(Direction),cos(Direction));
        // ROS_INFO("x = %d posx = %d ,float(Forward) / float(Omega) * (sin(Direction) - sin(Theta)) = %f, float(Side) / Omega * (cos(-Theta) - cos(Dir2)) = %f",x,posx,float(Forward) / float(Omega) * (sin(Direction) - sin(Theta)),float(Side) / Omega * (cos(-Theta) - cos(Dir2)));
        // ROS_INFO("y = %d posy = %d ,float(Forward) / Omega * (cos(Theta) - cos(Direction)) = %f, float(Side) / Omega * (sin(-Theta) - sin(Dir2)) = %f",y,posy,float(Forward) / Omega * (cos(Theta) - cos(Direction)),float(Side) / Omega * (sin(-Theta) - sin(Dir2)));    
    }

    // ROS_INFO("x = %d, y = %d",x,y);
    if( x < 0 || x > 1100 ||
        y < 0 || y > 800 )
    {
        x = posx;
        y = posy;
    }

    p.pos.pose.x = x;
    p.pos.pose.y = y;
    float rotat = Direction * RAD2DEG;
    p.pos.angle = normalize_angle(rotat);
    // ROS_INFO("posx = %d ,posy = %d ,rotation = %f ",x,y,normalize_angle(rotat));
}

void ParticleFilter::GetUpBackUp()
{
    // ROS_INFO("GetUpBackUp");    
    // posx += int(gauss(7, 0));
    // posy += int(gauss(7, 0));
    // rotation += int(gauss(25, 0));
}

void ParticleFilter::GetUpFrontUp()
{   
    // ROS_INFO("GetUpFrontUp");    
    // posx += int(gauss(7,-30)*sin(rotation*DEG2RAD));
    // posy += int(gauss(7,-30)*cos(rotation*DEG2RAD));
    // rotation += int(gauss(25, 0));
    // GetUpBackUp();
}

void ParticleFilter::FindBestParticle(scan_line *feature_point_observation_data, LineINF *Line_observation_data)
{
    ROS_INFO("FindBestParticle");
    
    double totalweight = 0.0;
    double Lineweight = 0.0;
    double fwl = -1.0;
    // A_MCL (長期移動平均)
    float x_avg = 0.0;
    float y_avg = 0.0;
    for(int i = 0; i < particlepoint_num; i++)
    {
        double factorweight = 1.0;
        x_avg += (float)particlepoint[i].pos.pose.x;
        y_avg += (float)particlepoint[i].pos.pose.y;
        // ROS_INFO("particlepoint[%d].pos = %d %d %f",i,particlepoint[i].pos.pose.x,particlepoint[i].pos.pose.y,particlepoint[i].pos.angle);
        if(use_feature_point)
        {
            particlepoint[i].fitness_value = 0.0;
            particlepoint[i].weight = 0.0;
            int real_feature_point_cnt = 0;
            for(int j = 0; j < particlepoint[i].featurepoint_scan_line.size(); j++)//36
            {
                real_feature_point_cnt += (*(feature_point_observation_data + j)).feature_point.size();
                // ROS_INFO("(*(feature_point_observation_data + j)).feature_point.size() = %d",(*(feature_point_observation_data + j)).feature_point.size());
                // ROS_INFO("particlepoint[i].featurepoint_scan_line[j].feature_point.size() = %d",particlepoint[i].featurepoint_scan_line[j].feature_point.size());

                if(particlepoint[i].featurepoint_scan_line[j].feature_point.size() == (*(feature_point_observation_data + j)).feature_point.size())
                {
                    int scan_line_fitness = 0.0;
                    for(int k = 0; k < particlepoint[i].featurepoint_scan_line[j].feature_point.size(); k++)
                    {
                        if((*(feature_point_observation_data + j)).feature_point[k].dis < 0)
                        {
                            if(particlepoint[i].featurepoint_scan_line[j].feature_point[k].dis < 0)
                            {
                                scan_line_fitness += MAP_MAX_LENGTH;
                            }
                        }
                        else
                        {
                            if(particlepoint[i].featurepoint_scan_line[j].feature_point[k].dis > 0)
                            {
                                int Dis_error = abs((*(feature_point_observation_data + j)).feature_point[k].dis - particlepoint[i].featurepoint_scan_line[j].feature_point[k].dis);
                                Dis_error = abs(MAP_MAX_LENGTH - Dis_error);
                                scan_line_fitness = scan_line_fitness + Dis_error;
                            }
                        }
                    }
                    particlepoint[i].fitness_value += ((float)scan_line_fitness / (float)particlepoint[i].featurepoint_scan_line[j].feature_point.size());
                }
                else if(particlepoint[i].featurepoint_scan_line[j].feature_point.size() > (*(feature_point_observation_data + j)).feature_point.size())
                {
                    int scan_line_fitness = 0;
                    for(int k = 0; k < (*(feature_point_observation_data + j)).feature_point.size(); k++)
                    {
                        if((*(feature_point_observation_data + j)).feature_point[k].dis < 0)
                        {
                            if(particlepoint[i].featurepoint_scan_line[j].feature_point[k].dis < 0)
                            {
                                scan_line_fitness += MAP_MAX_LENGTH;
                            }
                        }
                        else
                        {
                            if(particlepoint[i].featurepoint_scan_line[j].feature_point[k].dis > 0)
                            {
                                int Dis_error = abs((*(feature_point_observation_data + j)).feature_point[k].dis - particlepoint[i].featurepoint_scan_line[j].feature_point[k].dis);
                                Dis_error = abs(MAP_MAX_LENGTH - Dis_error);
                                scan_line_fitness = scan_line_fitness + Dis_error;
                            }
                        }
                    }
                    particlepoint[i].fitness_value += (float)((float)scan_line_fitness / (float)particlepoint[i].featurepoint_scan_line[j].feature_point.size());
                }
                else
                {
                    int scan_line_fitness = 0;
                    for(int k = 0; k < particlepoint[i].featurepoint_scan_line[j].feature_point.size(); k++)
                    {
                        if((*(feature_point_observation_data + j)).feature_point[k].dis < 0)
                        {
                            if(particlepoint[i].featurepoint_scan_line[j].feature_point[k].dis < 0)
                            {
                                scan_line_fitness += MAP_MAX_LENGTH;
                            }
                        }
                        else
                        {
                            if(particlepoint[i].featurepoint_scan_line[j].feature_point[k].dis > 0)
                            {
                                int Dis_error = abs((*(feature_point_observation_data + j)).feature_point[k].dis - particlepoint[i].featurepoint_scan_line[j].feature_point[k].dis);
                                Dis_error = abs(MAP_MAX_LENGTH - Dis_error);
                                scan_line_fitness = scan_line_fitness + Dis_error;
                            }
                        }
                    }
                    particlepoint[i].fitness_value += (scan_line_fitness / (*(feature_point_observation_data + j)).feature_point.size());
                }
                // ROS_INFO("particlepoint[%d].fitness_value = %d",i,particlepoint[i].fitness_value);  
            }
            // ROS_INFO("real_feature_point_cnt = %d",real_feature_point_cnt);  
            particlepoint[i].weight = (float)particlepoint[i].fitness_value / ((float)(real_feature_point_cnt * MAP_MAX_LENGTH));
            // ROS_INFO("particlepoint[%d].weight = %f",i,particlepoint[i].weight);  
            // factorweight *= exp(fwl)/(sqrt(2*M_PI))*SigmaIMU;
            // particlepoint[i].wfactors = max(min(log(factorweight/particlepoint[i].weight),2.),0.);
        }
        //待修改
        Lineweight = 1.0/(float)particlepoint_num;
        if(use_lineinformation)
        {
            double dis_diff = 0.0;

            Eigen::Vector2d h;
            double likehoodtmp = 0.0;
            // ROS_INFO("particlepoint[%d].landmark_list = %d",i,particlepoint[i].landmark_list.size());
            // ROS_INFO("Line_observation_data_Size = %d",Line_observation_data_Size);
            int x = Line_observation_data_Size;
            for(int j = 0; j < Line_observation_data_Size; j++)
            {
                double maxscore = 0.0;
                double linemaxscore = 0.0;
                int ID = 0;
                // ROS_INFO("Line_observation_data [%d] = %f %f",j,(*(Line_observation_data + j)).distance,normalize_angle_RAD((*(Line_observation_data + j)).Line_theta));
                // if((*(Line_observation_data + j)).Line_length <= 0.5)
                // {
                //     x --;
                //     continue;
                // }
                bool first_loop_flag = true;
                int count = 0;
                for(int m = 0; m < particlepoint[i].landmark_list.size(); m++)//計算虛擬地圖中的地標相似性
                {
                    // ROS_INFO("======================");
                    double p = 0.0;
                    linemaxscore = 0.0;
                    
                    if(particlepoint[i].landmark_list[m].Nearest_point.x > 1100 || particlepoint[i].landmark_list[m].Nearest_point.x <= 0 || particlepoint[i].landmark_list[m].Nearest_point.y > 800 || particlepoint[i].landmark_list[m].Nearest_point.y <= 0 )
                    {
                        count ++;
                        continue;
                    }
                    if(particlepoint[i].landmark_list[m].obersvated == false)
                    {
                        //initialize the ekf for this landmark    
                        // ROS_INFO("false");
                        Eigen::Matrix2d H;
                        Measurement_model(particlepoint[i], m, h, H);
                        // ROS_INFO("false");
                        particlepoint[i].landmark_list[m].sigma = (H.transpose()*R.inverse()*H).inverse();
                        // ROS_INFO("false");
                        // cout<<" 2 particlepoint[i].landmark_list[m].sigma "<< endl << particlepoint[i].landmark_list[m].sigma <<endl;
                        particlepoint[i].landmark_list[m].obersvated = true;
                        maxscore = 0.001;
                        count ++;
                    }else{
                        //get the Jacobian with respect to the landmark position
                        // ROS_INFO("true");
                        Eigen::Matrix2d G;
                        Eigen::Vector2d expect_Z;
                        Measurement_model(particlepoint[i], m, expect_Z, G);
                        // ROS_INFO("true");
                        Eigen::Matrix2d sig = particlepoint[i].landmark_list[m].sigma;
                        //compute the measurement covariance
                        Eigen::Matrix2d Z   = G * sig * G.transpose() + R;
                        //calculate the Kalman gain K
                        Eigen::Matrix2d K = sig * G.transpose() * Z.inverse();
                        //calculat the error between the z and expected Z
                        Eigen::Vector2d z_actual;
                        // z_actual << 0, 0;
                        z_actual << (*(Line_observation_data + j)).distance, normalize_angle_RAD((*(Line_observation_data + j)).Line_theta);
                        Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
                        Eigen::Vector2d z_diff = z_actual - expect_Z;
                        z_diff(1) = AngleDiff(z_diff(1));
                        double p = exp(-0.5*z_diff.transpose()*Z.inverse()*z_diff)/sqrt(2 * M_PI * Z.determinant());
                        // ROS_INFO(" p = %f",p);
                        // ROS_INFO("true");
                        
                        if(particlepoint[i].landmark_list[m].Nearest_point != Point(0,0) && first_loop_flag)
                        {
                            first_loop_flag = false;
                            maxscore = p;
                            ID = m;
                        }

                        if(maxscore < p && !first_loop_flag)
                        {
                            maxscore = p;
                            ID = m; 
                        }  
                        
                        count ++;
                        if(count == particlepoint[i].landmark_list.size()-1)
                        {
                            Eigen::Matrix2d G;
                            Eigen::Vector2d expect_Z;
                            Measurement_model(particlepoint[i], ID, expect_Z, G);
                            Eigen::Matrix2d sig = particlepoint[i].landmark_list[ID].sigma;
                            //compute the measurement covariance
                            Eigen::Matrix2d Z   = G * sig * G.transpose() + R;
                            //calculate the Kalman gain K
                            Eigen::Matrix2d K = sig * G.transpose() * Z.inverse();
                            particlepoint[i].landmark_list[ID].sigma = (I - K * G)*particlepoint[i].landmark_list[ID].sigma;
                            particlepoint[i].landmark_list[ID].realdistance = dis_diff;
                        }                                           
                    }                                      
                }
                
                ROS_INFO(" maxscore[%d] = %f",ID,maxscore);
                likehoodtmp += maxscore;
                ROS_INFO(" likehoodtmp = %f",likehoodtmp);
                // ROS_INFO("particlepoint[%d].wfactors = %f",i,particlepoint[i].wfactors);           
            
            }
            // ROS_INFO("--------------------------");
            Lineweight = (0.001) * likehoodtmp;

            // particlepoint[i].weight = particlepoint[i].weight * likehoodtmp;
            ROS_INFO("Lineweight[%d] = %f",i,Lineweight);  
            
        }
        factorweight *= exp(fwl);
        // ROS_INFO("----factorweight[%d] = %f----",i,factorweight);
        // ROS_INFO("=====particlepoint[%d].weight = %f=====",i,1.0-particlepoint[i].weight);
        if(use_feature_point && use_lineinformation)
        {
            particlepoint[i].weight = particlepoint[i].weight - Lineweight;
        }else
        {
            particlepoint[i].weight = 1.0 - Lineweight;
        }
        
        if(particlepoint[i].weight == 0.0)
        {
            particlepoint[i].weight = 0.95;
        }
        particlepoint[i].wfactors = max(min(log(factorweight/(1.0-particlepoint[i].weight)),2.),0.);
        ROS_INFO("----particlepoint[%d].wfactors = %f----",i,particlepoint[i].wfactors);
        ROS_INFO("----particlepoint[%d].weight = %f----",i,1.0-particlepoint[i].weight);
        totalweight += particlepoint[i].weight;
    }

    totalweights = totalweight;

    x_avg = x_avg / (float)particlepoint_num;
    y_avg = y_avg / (float)particlepoint_num;

    particlepoint_compare = particlepoint;
    sort(particlepoint_compare.begin(), particlepoint_compare.end(), greater<ParticlePoint>());
    FindRobotPosition(x_avg, y_avg);
    // ROS_INFO("weight:%f", particlepoint_compare[0].weight);
    // ROS_INFO("weight_low:%f", particlepoint[particlepoint.size() - 1].weight);
    // ROS_INFO("x:%d", Robot_Position.pos.pose.x);
    // ROS_INFO("y:%d", Robot_Position.pos.pose.y);
    //-------------判斷適應值太高的狀況就不更新---------
    if(particlepoint_compare[0].weight < 0.28)
    {
        find_best_flag = false;
    }
    else
    {
        find_best_flag = true;
    }
    //-------------判斷適應值太高的狀況就不更新---------
}

int ParticleFilter::TournamentResample(int excellent_particle_num)
{
    // ROS_INFO("TournamentResample");
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

void ParticleFilter::GetBestPoseAndLandmark(VectorXd& mu_ )
{
    // int index = TournamentResample(excellent_particle_num);
    // ParticlePoint& current_particle = particlepoint[index];
    // int N_ = current_particle.landmark_list.size();
      
    // mu_ = Eigen::VectorXd::Zero(2 * N_ + 3);
    // mu_(0) = current_particle.pos.angle;
    // mu_(1) = current_particle.pos.pose.x;
    // mu_(2) = current_particle.pos.pose.y;

    // for (int i = 0; i < N_; i++) {
    //     mu_(2 * i + 3) = current_particle.landmark_list[i].mu(0);
    //     mu_(2 * i + 4) = current_particle.landmark_list[i].mu(1);
    // }
}


void ParticleFilter::CalcFOVArea(int focus, int top, int bottom, int top_width, int bottom_width, float horizontal_head_angle)
{
    ROS_INFO("CalcFOVArea");
    for(int i = 0; i < particlepoint_num; i++)
    {
        // ROS_INFO("i = %d",i);
        particlepoint[i].FOV_dir = normalize_angle(particlepoint[i].pos.angle + horizontal_head_angle);
        

        float HFOV_2D_bottom = atan2(bottom_width,bottom) * 180 / PI;
        HFOV_2D_bottom = normalize_angle(HFOV_2D_bottom);
        float HFOV_2D_top = atan2(top_width,top) * 180 / PI;
        HFOV_2D_top = normalize_angle(HFOV_2D_top);
        float top_waist_length = sqrt(pow(top,2) + pow(top_width,2));
        float bottom_waist_length = sqrt(pow(bottom,2) + pow(bottom_width,2));

        // //Camera_Focus.x = particlepoint[i].pos.pose.x + focus * cos(FOV_dir * DEG2RAD);
        // //Camera_Focus.y = particlepoint[i].pos.pose.y + focus * sin(FOV_dir * DEG2RAD);
        float right_sight_top_angle = normalize_angle(particlepoint[i].FOV_dir - HFOV_2D_top);
        float right_sight_bottom_angle = normalize_angle(particlepoint[i].FOV_dir - HFOV_2D_bottom);
        float left_sight_top_angle = normalize_angle(particlepoint[i].FOV_dir + HFOV_2D_top);
        float left_sight_bottom_angle = normalize_angle(particlepoint[i].FOV_dir + HFOV_2D_bottom);

        
        // float top_waist_length = sqrt(pow(image_top_length,2) + pow(image_top_width_length,2));
        // float bottom_waist_length = sqrt(pow(image_bottom_length,2) + pow(image_bottom_width_length,2));


        // ROS_INFO("image_bottom_angle = %f",image_bottom_angle); 

        // ROS_INFO("image_top_length = %f",image_top_length); 
        // ROS_INFO("image_bottom_length = %f",image_bottom_length); 
        // ROS_INFO("image_top_width_length = %f",image_top_width_length); 
        // ROS_INFO("image_bottom_width_length = %f",image_bottom_width_length); 
    
        // ROS_INFO("top_waist_length = %f",top_waist_length); 
        // ROS_INFO("bottom_waist_length = %f",bottom_waist_length); 

        // ROS_INFO("HFOV_2D_bottom = %f",HFOV_2D_bottom); 
        // ROS_INFO("HFOV_2D_top = %f",HFOV_2D_top); 
        
        // ROS_INFO("right_sight_top_angle = %f",right_sight_top_angle); 
        // ROS_INFO("right_sight_bottom_angle = %f",right_sight_bottom_angle); 
        // ROS_INFO("left_sight_top_angle = %f",left_sight_top_angle); 
        // ROS_INFO("left_sight_bottom_angle = %f",left_sight_bottom_angle); 

        particlepoint[i].FOV_Bottom_Right.x = particlepoint[i].pos.pose.x + bottom_waist_length * cos(right_sight_bottom_angle * DEG2RAD);
        particlepoint[i].FOV_Bottom_Right.x = Frame_Area(particlepoint[i].FOV_Bottom_Right.x, MAP_LENGTH);
        particlepoint[i].FOV_Bottom_Right.y = particlepoint[i].pos.pose.y - bottom_waist_length * sin(right_sight_bottom_angle * DEG2RAD);
        particlepoint[i].FOV_Bottom_Right.y = Frame_Area(particlepoint[i].FOV_Bottom_Right.y, MAP_WIDTH);

        particlepoint[i].FOV_Top_Right.x = particlepoint[i].pos.pose.x + top_waist_length * cos(right_sight_top_angle * DEG2RAD);
        // particlepoint[i].FOV_Top_Right.x = Frame_Area(particlepoint[i].FOV_Top_Right.x, MAP_LENGTH);
        particlepoint[i].FOV_Top_Right.y = particlepoint[i].pos.pose.y - top_waist_length * sin(right_sight_top_angle * DEG2RAD);   
        // particlepoint[i].FOV_Top_Right.y = Frame_Area(particlepoint[i].FOV_Top_Right.y, MAP_WIDTH);


        particlepoint[i].FOV_Bottom_Left.x = particlepoint[i].pos.pose.x + bottom_waist_length * cos(left_sight_bottom_angle * DEG2RAD);
        particlepoint[i].FOV_Bottom_Left.x = Frame_Area(particlepoint[i].FOV_Bottom_Left.x, MAP_LENGTH);
        particlepoint[i].FOV_Bottom_Left.y = particlepoint[i].pos.pose.y - bottom_waist_length * sin(left_sight_bottom_angle * DEG2RAD); 
        particlepoint[i].FOV_Bottom_Left.y = Frame_Area(particlepoint[i].FOV_Bottom_Left.y, MAP_WIDTH); 

        particlepoint[i].FOV_Top_Left.x = particlepoint[i].pos.pose.x + top_waist_length * cos(left_sight_top_angle * DEG2RAD);
        // particlepoint[i].FOV_Top_Left.x = Frame_Area(particlepoint[i].FOV_Top_Left.x, MAP_LENGTH);
        particlepoint[i].FOV_Top_Left.y = particlepoint[i].pos.pose.y - top_waist_length * sin(left_sight_top_angle * DEG2RAD);
        // particlepoint[i].FOV_Top_Left.y = Frame_Area(particlepoint[i].FOV_Top_Left.y, MAP_WIDTH);
        // ROS_INFO("-----FOV_Top_Right = %d %d",Robot_Position.FOV_Top_Right.x,Robot_Position.FOV_Top_Right.y); 
        // ROS_INFO("FOV_Top_Left = %d %d",Robot_Position.FOV_Top_Left.x,Robot_Position.FOV_Top_Left.y); 
        particlepoint[i].FOV_tmp = Point(-1,-1);
        particlepoint[i].FOV_tmp2 = Point(-1,-1);
        Point range = Point(MAP_LENGTH,MAP_WIDTH);
        // ROS_INFO("CheckPointArea(&particlepoint[i],range.x-1,range.y-1,4)"); 
        if(CheckPointArea(&particlepoint[i],range.x-1,range.y-1,4))
        {
            int disFOV_Top_R = sqrt(pow(particlepoint[i].FOV_Top_Right.x - range.x-1,2) + pow(particlepoint[i].FOV_Top_Right.y - range.y-1,2));
            int disFOV_Top_L = sqrt(pow(particlepoint[i].FOV_Top_Left.x - range.x-1,2) + pow(particlepoint[i].FOV_Top_Left.y - range.y-1,2));
            if(disFOV_Top_R < disFOV_Top_L)
            {
                particlepoint[i].FOV_tmp = Point(range.x-1,range.y-1);
            }else{
                particlepoint[i].FOV_tmp2 = Point(range.x-1,range.y-1);
            }
        }
        // ROS_INFO("CheckPointArea(&particlepoint[i],0,range.y-1,4)"); 
        if(CheckPointArea(&particlepoint[i],0,range.y-1,4))
        {
            int disFOV_Top_R = sqrt(pow(particlepoint[i].FOV_Top_Right.x,2) + pow(particlepoint[i].FOV_Top_Right.y - range.y-1,2));
            int disFOV_Top_L = sqrt(pow(particlepoint[i].FOV_Top_Left.x,2) + pow(particlepoint[i].FOV_Top_Left.y - range.y-1,2));
            if(disFOV_Top_R < disFOV_Top_L)
            {
                particlepoint[i].FOV_tmp = Point(0,range.y-1);
            }else{
                particlepoint[i].FOV_tmp2 = Point(0,range.y-1);
            }
        }
        // ROS_INFO("CheckPointArea(&particlepoint[i],range.x-1,0,4)"); 
        if(CheckPointArea(&particlepoint[i],range.x-1,0,4))
        {
            int disFOV_Top_R = sqrt(pow(particlepoint[i].FOV_Top_Right.x - range.x-1,2) + pow(particlepoint[i].FOV_Top_Right.y,2));
            int disFOV_Top_L = sqrt(pow(particlepoint[i].FOV_Top_Left.x - range.x-1,2) + pow(particlepoint[i].FOV_Top_Left.y,2));
            if(disFOV_Top_R < disFOV_Top_L)
            {
                particlepoint[i].FOV_tmp = Point(range.x-1,0);
            }else{
                particlepoint[i].FOV_tmp2 = Point(range.x-1,0);
            }
        }
        // ROS_INFO("CheckPointArea(&particlepoint[i],0,0,4)"); 
        if(CheckPointArea(&particlepoint[i],0,0,4))
        {
            int disFOV_Top_R = sqrt(pow(particlepoint[i].FOV_Top_Right.x,2) + pow(particlepoint[i].FOV_Top_Right.y,2));
            int disFOV_Top_L = sqrt(pow(particlepoint[i].FOV_Top_Left.x,2) + pow(particlepoint[i].FOV_Top_Left.y,2));
            if(disFOV_Top_R < disFOV_Top_L)
            {
                particlepoint[i].FOV_tmp = Point(0,0);
            }else{
                particlepoint[i].FOV_tmp2 = Point(0,0);
            }
        }
        // ROS_INFO("======FOV_Top_Right = %d %d",particlepoint[i].FOV_Top_Right.x,particlepoint[i].FOV_Top_Right.y); 
        // ROS_INFO("FOV_Top_Left = %d %d",particlepoint[i].FOV_Top_Left.x,particlepoint[i].FOV_Top_Left.y);
        // ROS_INFO("Frame_Area_2"); 
        particlepoint[i].FOV_Top_Right = Frame_Area_2(particlepoint[i].FOV_Bottom_Right, particlepoint[i].FOV_Top_Right ,range);
        particlepoint[i].FOV_Top_Left = Frame_Area_2(particlepoint[i].FOV_Bottom_Left, particlepoint[i].FOV_Top_Left ,range);
        
        // ROS_INFO("FOV_Bottom_Right = %d %d",particlepoint[i].FOV_Bottom_Right.x,particlepoint[i].FOV_Bottom_Right.y); 
        // ROS_INFO("FOV_Top_Right = %d %d",particlepoint[i].FOV_Top_Right.x,particlepoint[i].FOV_Top_Right.y); 
        // ROS_INFO("FOV_Top_Left = %d %d",particlepoint[i].FOV_Top_Left.x,particlepoint[i].FOV_Top_Left.y); 
        // ROS_INFO("-----FOV_Bottom_Left = %d %d",particlepoint[i].FOV_Bottom_Left.x,particlepoint[i].FOV_Bottom_Left.y);    
        // ROS_INFO("FOV_corrdinate"); 
        particlepoint[i].FOV_corrdinate[0].x = particlepoint[i].FOV_Top_Left.x;
        particlepoint[i].FOV_corrdinate[0].y = particlepoint[i].FOV_Top_Left.y;
        particlepoint[i].FOV_corrdinate[1].x = particlepoint[i].FOV_tmp2.x;
        particlepoint[i].FOV_corrdinate[1].y = particlepoint[i].FOV_tmp2.y;
        particlepoint[i].FOV_corrdinate[2].x = particlepoint[i].FOV_tmp.x;
        particlepoint[i].FOV_corrdinate[2].y = particlepoint[i].FOV_tmp.y;
        particlepoint[i].FOV_corrdinate[3].x = particlepoint[i].FOV_Top_Right.x;
        particlepoint[i].FOV_corrdinate[3].y = particlepoint[i].FOV_Top_Right.y;
        particlepoint[i].FOV_corrdinate[4].x = particlepoint[i].FOV_Bottom_Right.x;
        particlepoint[i].FOV_corrdinate[4].y = particlepoint[i].FOV_Bottom_Right.y;
        particlepoint[i].FOV_corrdinate[5].x = particlepoint[i].FOV_Bottom_Left.x;
        particlepoint[i].FOV_corrdinate[5].y = particlepoint[i].FOV_Bottom_Left.y;
        // ROS_INFO("finish");  
    }
}

void ParticleFilter::FindLandMarkInVirtualField(ParticlePoint *particlepoint)
{
    // ROS_INFO("FindLandMarkInVirtualField");
    
    vector<Point> tmp;
    tmp.clear();
    // ROS_INFO("particlepoint->FOV_corrdinate[0] = %d %d",particlepoint->FOV_corrdinate[0].x,particlepoint->FOV_corrdinate[0].y);
    // ROS_INFO("particlepoint->FOV_corrdinate[1] = %d %d",particlepoint->FOV_corrdinate[1].x,particlepoint->FOV_corrdinate[1].y);
    // ROS_INFO("particlepoint->FOV_corrdinate[2] = %d %d",particlepoint->FOV_corrdinate[2].x,particlepoint->FOV_corrdinate[2].y);
    if((particlepoint->FOV_corrdinate[1].x == -1 && particlepoint->FOV_corrdinate[1].y == -1) &&( particlepoint->FOV_corrdinate[2].x != -1 && particlepoint->FOV_corrdinate[2].y != -1 ))
    {
        tmp = {particlepoint->FOV_corrdinate[0],particlepoint->FOV_corrdinate[2],particlepoint->FOV_corrdinate[3],particlepoint->FOV_corrdinate[4],particlepoint->FOV_corrdinate[5]};
        // ROS_INFO("1.tmp.size() = %d",tmp.size());
    }else if((particlepoint->FOV_corrdinate[1].x != -1 && particlepoint->FOV_corrdinate[1].y != -1) && particlepoint->FOV_corrdinate[2].x == -1 && particlepoint->FOV_corrdinate[2].y == -1 )
    {
        tmp = {particlepoint->FOV_corrdinate[0],particlepoint->FOV_corrdinate[1],particlepoint->FOV_corrdinate[3],particlepoint->FOV_corrdinate[4],particlepoint->FOV_corrdinate[5]};
        // ROS_INFO("2.tmp.size() = %d",tmp.size());
    }else if(particlepoint->FOV_corrdinate[1].x == -1 && particlepoint->FOV_corrdinate[1].y == -1 && particlepoint->FOV_corrdinate[2].x == -1 && particlepoint->FOV_corrdinate[2].y == -1 )
    {
        tmp = {particlepoint->FOV_corrdinate[0],particlepoint->FOV_corrdinate[3],particlepoint->FOV_corrdinate[4],particlepoint->FOV_corrdinate[5]};
        // ROS_INFO("3.tmp.size() = %d",tmp.size());
    }else{
        tmp = {particlepoint->FOV_corrdinate[0],particlepoint->FOV_corrdinate[1],particlepoint->FOV_corrdinate[2],particlepoint->FOV_corrdinate[3],particlepoint->FOV_corrdinate[4],particlepoint->FOV_corrdinate[5]};
        // ROS_INFO("4.tmp.size() = %d",tmp.size());
    }

    // ROS_INFO("tmp.size() = %d",tmp.size());
    FieldLine_data FOV_function_data_tmp[tmp.size()]; // To caculate FOV's linear function
    
    for(int j = 0; j < tmp.size(); j++)
    {       
        Point FOV_tmp = tmp[j];
        Point FOV_tmp2 = tmp[0];
        Point FOV_tmp1 = Point(0,0);

        int x1 = 0;
        int y1 = 0;
        int x2 = 0;
        int y2 = 0;
        if(j<tmp.size()-1)
        {
            FOV_tmp1 = tmp[j+1];
            x1 = FOV_tmp.x;
            y1 = FOV_tmp.y;
            x2 = FOV_tmp1.x;
            y2 = FOV_tmp1.y;
        }else{
            x1 = FOV_tmp.x;
            y1 = FOV_tmp.y;
            x2 = FOV_tmp2.x;
            y2 = FOV_tmp2.y;
        }
        // ROS_INFO("x1 = %d. y1 = %d, x2 = %d, y2 = %d",x1,y1,x2,y2);
        if((y1-y2) == 0)
        {
            if(x1<x2)
            {
                FOV_function_data_tmp[j].start_point = Point(x1,y1);
                FOV_function_data_tmp[j].end_point = Point(x2,y2);
            }else{
                FOV_function_data_tmp[j].start_point = Point(x2,y2);
                FOV_function_data_tmp[j].end_point = Point(x1,y1);
            }
        }else{
            if(y1<y2)
            {
                FOV_function_data_tmp[j].start_point = Point(x1,y1);
                FOV_function_data_tmp[j].end_point = Point(x2,y2);
            }else{
                FOV_function_data_tmp[j].start_point = Point(x2,y2);
                FOV_function_data_tmp[j].end_point = Point(x1,y1);
            }
        }  
        // ROS_INFO("FOV_function_data_tmp[%d].start_point = %d,%d FOV_function_data_tmp[%d].end_point = %d,%d",
        // j,FOV_function_data_tmp[j].start_point.x,FOV_function_data_tmp[j].start_point.y,
        // j,FOV_function_data_tmp[j].end_point.x,FOV_function_data_tmp[j].end_point.y);  
    }
    
    vector<Point> intersect_point_list;
    vector<Point> virtualpointinFOV;
    vector<Vec4i> detect_line_list;
    vector<int> tmp_ID;
    intersect_point_list.clear();
    virtualpointinFOV.clear();
    detect_line_list.clear(); 
    tmp_ID.clear();
    for(int k = 0; k < field_list.size(); k++) // find the line in FOV (virtual field)
    {
        int ID = field_list[k].Line_ID;
        Vec4i X = {field_list[k].start_point.x,field_list[k].start_point.y,field_list[k].end_point.x,field_list[k].end_point.y};
        int startpointinarea = CheckPointArea(particlepoint, X[0], X[1],tmp.size());
        int endpointinarea = CheckPointArea(particlepoint, X[2], X[3],tmp.size());
        // ROS_INFO("field_list[%d] = %d, %d %d %d",k,X[0],X[1],X[2],X[3]);
        // ROS_INFO("startpointinarea = %d, endpointinarea = %d",startpointinarea,endpointinarea);
        if(startpointinarea == 1 && endpointinarea == 1)
        {
            Vec4i tmpx = {0,0,0,0};
            tmpx = {X[0], X[1] ,X[2] ,X[3]};
            tmp_ID.push_back(ID);
            detect_line_list.push_back(tmpx);
            continue;
        }else if(startpointinarea == 1 && endpointinarea != 1)
        {
            virtualpointinFOV.push_back(Point(X[0], X[1]));
        }else if(startpointinarea != 1 && endpointinarea == 1)
        {
            virtualpointinFOV.push_back(Point(X[2], X[3]));
        }

        int count;
        for(int l = 0; l < tmp.size(); l++)
        {
            Vec4i Y = {FOV_function_data_tmp[l].start_point.x,FOV_function_data_tmp[l].start_point.y,FOV_function_data_tmp[l].end_point.x,FOV_function_data_tmp[l].end_point.y};
            count = 0;
            Vec4i tmpx = {0,0,0,0};
            Vec4i tmp1 = {FOV_function_data_tmp[tmp.size()-2].start_point.x,FOV_function_data_tmp[tmp.size()-2].start_point.y,FOV_function_data_tmp[tmp.size()-2].end_point.x,FOV_function_data_tmp[tmp.size()-2].end_point.y};
            if(intersect(X,Y) == 1)//相交
            {
                Point intersectpoint = IntersectPoint(X,Y);
                if(virtualpointinFOV.size() == 1)
                {                   
                    if(MinDistance(tmp1,intersectpoint) < MinDistance(tmp1,virtualpointinFOV[0]))
                    {
                        tmpx = {intersectpoint.x ,intersectpoint.y,virtualpointinFOV[0].x, virtualpointinFOV[0].y};
                        // ROS_INFO("%d %d %d %d",tmp[0],tmp[1],tmp[2],tmp[3]);
                    }else{
                        tmpx = {virtualpointinFOV[0].x, virtualpointinFOV[0].y ,intersectpoint.x ,intersectpoint.y};
                        // ROS_INFO("%d %d %d %d",tmp[0],tmp[1],tmp[2],tmp[3]);
                    }
                    tmp_ID.push_back(ID);
                    detect_line_list.push_back(tmpx);
                    virtualpointinFOV.clear();
                    continue;
                }
                intersect_point_list.push_back(intersectpoint);
                // ROS_INFO("intersectpoint = %d,%d",intersectpoint.x,intersectpoint.y);
            }else if(intersect(X,Y) == 2)//重合
            {
                if(MinDistance(tmp1,Point(Y[0],Y[1])) < MinDistance(tmp1,Point(Y[2],Y[3])))
                {
                    tmpx = {Y[0],Y[1],Y[2],Y[3]};
                    // ROS_INFO("%d %d %d %d",tmp[0],tmp[1],tmp[2],tmp[3]);
                }else{
                    tmpx = {Y[2],Y[3],Y[1],Y[2]};
                    // ROS_INFO("%d %d %d %d",tmp[0],tmp[1],tmp[2],tmp[3]);
                }
                tmp_ID.push_back(ID);
                detect_line_list.push_back(tmpx);
                intersect_point_list.clear();
                virtualpointinFOV.clear();
            }else{
                
                count++;
            }  
        }
        if(virtualpointinFOV.size() == 0) //FOV中無末端點
        {
            if(intersect_point_list.size() == 2)
            {
                Vec4i tmpx = {0,0,0,0};
                Vec4i tmp1 = {FOV_function_data_tmp[tmp.size()-2].start_point.x,FOV_function_data_tmp[tmp.size()-2].start_point.y,FOV_function_data_tmp[tmp.size()-2].end_point.x,FOV_function_data_tmp[tmp.size()-2].end_point.y};
                if(MinDistance(tmp1,intersect_point_list[0]) < MinDistance(tmp1,intersect_point_list[1]))
                {
                    tmpx = {intersect_point_list[0].x,intersect_point_list[0].y,intersect_point_list[1].x,intersect_point_list[1].y};
                    // ROS_INFO("%d %d %d %d",tmp[0],tmp[1],tmp[2],tmp[3]);
                }else{
                    tmpx = {intersect_point_list[1].x,intersect_point_list[1].y,intersect_point_list[0].x,intersect_point_list[0].y};
                    // ROS_INFO("%d %d %d %d",tmp[0],tmp[1],tmp[2],tmp[3]);
                }
                tmp_ID.push_back(ID);
                detect_line_list.push_back(tmpx);
                intersect_point_list.clear();
            }else{
                intersect_point_list.clear();
            }
        }
    }
    
    for(int n = 0; n < field_list.size();n ++)
    {
        particlepoint->landmark_list[n].Nearest_point = Point(-1,-1);
    }
    for(int m = 0; m < detect_line_list.size(); m++)
    {
        LineINF landmark_tmp;
        int ID = tmp_ID[m];
        Vec4i tmpY = detect_line_list[m];
        // ROS_INFO("tmp = %d %d %d %d",tmp[0],tmp[1],tmp[2],tmp[3]);
        landmark_tmp = LineInformation(Point(tmpY[0],tmpY[1]),Point(tmpY[2],tmpY[3]),FOV_function_data_tmp[tmp.size()-2].start_point,FOV_function_data_tmp[tmp.size()-2].end_point);

        if(particlepoint->landmark_list[ID].obersvated == true)
        {
            particlepoint->landmark_list[ID].obersvated = true;
        }else{
            particlepoint->landmark_list[ID].obersvated = false;
        } 
        // ROS_INFO("landmark_tmp.Nearest_point[%d] = %d %d ",ID,landmark_tmp.Nearest_point.x,landmark_tmp.Nearest_point.y); 
        particlepoint->landmark_list[ID].Nearest_point = landmark_tmp.Nearest_point;
        particlepoint->landmark_list[ID].start_point = landmark_tmp.start_point;
        particlepoint->landmark_list[ID].Line_length = landmark_tmp.Line_length;
        particlepoint->landmark_list[ID].end_point = landmark_tmp.end_point;
        particlepoint->landmark_list[ID].Line_theta = landmark_tmp.Line_theta;
    }
}

void ParticleFilter::LandMarkMode(Landmarkmode mode)
{
    ROS_INFO("LandMarkMode");
    switch(mode)
    {
        case Landmarkmode::PARTICLEPIONT:
            for(int i = 0; i < particlepoint_num; i++)
            {
                FindLandMarkInVirtualField(&particlepoint[i]);
            }
            break;
        case Landmarkmode::ROBOT:
            FindLandMarkInVirtualField(&Robot_Position);
            break;
    }
}

void ParticleFilter::CalcFOVArea_averagepos(int focus, int top, int bottom, int top_width, int bottom_width, float horizontal_head_angle)
{
    // ROS_INFO("CalcFOVArea_averagepos");
    Robot_Position.FOV_dir = normalize_angle(Robot_Position.pos.angle + horizontal_head_angle);
    float HFOV_2D_bottom = atan2(bottom_width,bottom) * 180 / PI;
    HFOV_2D_bottom = normalize_angle(HFOV_2D_bottom);
    float HFOV_2D_top = atan2(top_width,top) * 180 / PI;
    HFOV_2D_top = normalize_angle(HFOV_2D_top);
    float top_waist_length = sqrt(pow(top,2) + pow(top_width,2));
    float bottom_waist_length = sqrt(pow(bottom,2) + pow(bottom_width,2));

    //Camera_Focus.x = Robot_Position.pos.pose.x + focus * cos(FOV_dir * DEG2RAD);
    //Camera_Focus.y = Robot_Position.pos.pose.y + focus * sin(FOV_dir * DEG2RAD);

    float right_sight_top_angle = normalize_angle(Robot_Position.FOV_dir - HFOV_2D_top);
    float right_sight_bottom_angle = normalize_angle(Robot_Position.FOV_dir - HFOV_2D_bottom);
    float left_sight_top_angle = normalize_angle(Robot_Position.FOV_dir + HFOV_2D_top);
    float left_sight_bottom_angle = normalize_angle(Robot_Position.FOV_dir + HFOV_2D_bottom);
        
    
    Robot_Position.FOV_Bottom_Right.x = Robot_Position.pos.pose.x + bottom_waist_length * cos(right_sight_bottom_angle * DEG2RAD);
    Robot_Position.FOV_Bottom_Right.x = Frame_Area(Robot_Position.FOV_Bottom_Right.x, MAP_LENGTH);
    Robot_Position.FOV_Bottom_Right.y = Robot_Position.pos.pose.y - bottom_waist_length * sin(right_sight_bottom_angle * DEG2RAD);
    Robot_Position.FOV_Bottom_Right.y = Frame_Area(Robot_Position.FOV_Bottom_Right.y, MAP_WIDTH);
    
    Robot_Position.FOV_Top_Right.x = Robot_Position.pos.pose.x + top_waist_length * cos(right_sight_top_angle * DEG2RAD);
    // Robot_Position.FOV_Top_Right.x = Frame_Area(Robot_Position.FOV_Top_Right.x, MAP_LENGTH);
    Robot_Position.FOV_Top_Right.y = Robot_Position.pos.pose.y - top_waist_length * sin(right_sight_top_angle * DEG2RAD);   
    // Robot_Position.FOV_Top_Right.y = Frame_Area(Robot_Position.FOV_Top_Right.y, MAP_WIDTH);
    
    Robot_Position.FOV_Bottom_Left.x = Robot_Position.pos.pose.x + bottom_waist_length * cos(left_sight_bottom_angle * DEG2RAD);
    Robot_Position.FOV_Bottom_Left.x = Frame_Area(Robot_Position.FOV_Bottom_Left.x, MAP_LENGTH);
    Robot_Position.FOV_Bottom_Left.y = Robot_Position.pos.pose.y - bottom_waist_length * sin(left_sight_bottom_angle * DEG2RAD); 
    Robot_Position.FOV_Bottom_Left.y = Frame_Area(Robot_Position.FOV_Bottom_Left.y, MAP_WIDTH); 
    Robot_Position.FOV_Top_Left.x = Robot_Position.pos.pose.x + top_waist_length * cos(left_sight_top_angle * DEG2RAD);
    // Robot_Position.FOV_Top_Left.x = Frame_Area(Robot_Position.FOV_Top_Left.x, MAP_LENGTH);
    Robot_Position.FOV_Top_Left.y = Robot_Position.pos.pose.y - top_waist_length * sin(left_sight_top_angle * DEG2RAD);
    // Robot_Position.FOV_Top_Left.y = Frame_Area(Robot_Position.FOV_Top_Left.y, MAP_WIDTH);
    
    Robot_Position.FOV_tmp = Point(-1,-1);
    Robot_Position.FOV_tmp2 = Point(-1,-1);
    Point range = Point(MAP_LENGTH,MAP_WIDTH);

    if(CheckPointArea(&Robot_Position,range.x-1,range.y-1,4))
    {
        int disFOV_Top_R = sqrt(pow(Robot_Position.FOV_Top_Right.x - range.x-1,2) + pow(Robot_Position.FOV_Top_Right.y - range.y-1,2));
        int disFOV_Top_L = sqrt(pow(Robot_Position.FOV_Top_Left.x - range.x-1,2) + pow(Robot_Position.FOV_Top_Left.y - range.y-1,2));
        if(disFOV_Top_R < disFOV_Top_L)
        {
            Robot_Position.FOV_tmp = Point(range.x-1,range.y-1);
        }else{
            Robot_Position.FOV_tmp2 = Point(range.x-1,range.y-1);
        }
    }
    if(CheckPointArea(&Robot_Position,0,range.y-1,4))
    {
        int disFOV_Top_R = sqrt(pow(Robot_Position.FOV_Top_Right.x,2) + pow(Robot_Position.FOV_Top_Right.y - range.y-1,2));
        int disFOV_Top_L = sqrt(pow(Robot_Position.FOV_Top_Left.x,2) + pow(Robot_Position.FOV_Top_Left.y - range.y-1,2));
        if(disFOV_Top_R < disFOV_Top_L)
        {
            Robot_Position.FOV_tmp = Point(0,range.y-1);
        }else{
            Robot_Position.FOV_tmp2 = Point(0,range.y-1);
        }
    }
    if(CheckPointArea(&Robot_Position,range.x-1,0,4))
    {
        int disFOV_Top_R = sqrt(pow(Robot_Position.FOV_Top_Right.x - range.x-1,2) + pow(Robot_Position.FOV_Top_Right.y,2));
        int disFOV_Top_L = sqrt(pow(Robot_Position.FOV_Top_Left.x - range.x-1,2) + pow(Robot_Position.FOV_Top_Left.y,2));
        if(disFOV_Top_R < disFOV_Top_L)
        {
            Robot_Position.FOV_tmp = Point(range.x-1,0);
        }else{
            Robot_Position.FOV_tmp2 = Point(range.x-1,0);
        }
    }
    if(CheckPointArea(&Robot_Position,0,0,4))
    {
        int disFOV_Top_R = sqrt(pow(Robot_Position.FOV_Top_Right.x,2) + pow(Robot_Position.FOV_Top_Right.y,2));
        int disFOV_Top_L = sqrt(pow(Robot_Position.FOV_Top_Left.x,2) + pow(Robot_Position.FOV_Top_Left.y,2));
        if(disFOV_Top_R < disFOV_Top_L)
        {
            Robot_Position.FOV_tmp = Point(0,0);
        }else{
            Robot_Position.FOV_tmp2 = Point(0,0);
        }
    }
    Robot_Position.FOV_Top_Right = Frame_Area_2(Robot_Position.FOV_Bottom_Right, Robot_Position.FOV_Top_Right ,range);
    Robot_Position.FOV_Top_Left = Frame_Area_2(Robot_Position.FOV_Bottom_Left, Robot_Position.FOV_Top_Left ,range);

    // ROS_INFO("image_bottom_angle = %f",image_bottom_angle); 

    // ROS_INFO("image_top_length = %f",image_top_length); 
    // ROS_INFO("image_bottom_length = %f",image_bottom_length); 
    // ROS_INFO("image_top_width_length = %f",image_top_width_length); 
    // ROS_INFO("image_bottom_width_length = %f",image_bottom_width_length); 
   
    // ROS_INFO("top_waist_length = %f",top_waist_length); 
    // ROS_INFO("bottom_waist_length = %f",bottom_waist_length); 


    // ROS_INFO("HFOV_2D_bottom = %f",HFOV_2D_bottom); 
    // ROS_INFO("HFOV_2D_top = %f",HFOV_2D_top); 
    
    // ROS_INFO("right_sight_top_angle = %f",right_sight_top_angle); 
    // ROS_INFO("right_sight_bottom_angle = %f",right_sight_bottom_angle); 
    // ROS_INFO("left_sight_top_angle = %f",left_sight_top_angle); 
    // ROS_INFO("left_sight_bottom_angle = %f",left_sight_bottom_angle); 

    // ROS_INFO("FOV_Bottom_Right = %d %d",Robot_Position.FOV_Bottom_Right.x,Robot_Position.FOV_Bottom_Right.y); 
    // ROS_INFO("FOV_Top_Right = %d %d",Robot_Position.FOV_Top_Right.x,Robot_Position.FOV_Top_Right.y); 
    // ROS_INFO("FOV_Top_Left = %d %d",Robot_Position.FOV_Top_Left.x,Robot_Position.FOV_Top_Left.y); 
    // ROS_INFO("FOV_Bottom_Left = %d %d",Robot_Position.FOV_Bottom_Left.x,Robot_Position.FOV_Bottom_Left.y);       
    
    Robot_Position.FOV_corrdinate[0].x = Robot_Position.FOV_Top_Left.x;
    Robot_Position.FOV_corrdinate[0].y = Robot_Position.FOV_Top_Left.y;
    Robot_Position.FOV_corrdinate[1].x = Robot_Position.FOV_tmp2.x;
    Robot_Position.FOV_corrdinate[1].y = Robot_Position.FOV_tmp2.y;
    Robot_Position.FOV_corrdinate[2].x = Robot_Position.FOV_tmp.x;
    Robot_Position.FOV_corrdinate[2].y = Robot_Position.FOV_tmp.y;
    Robot_Position.FOV_corrdinate[3].x = Robot_Position.FOV_Top_Right.x;
    Robot_Position.FOV_corrdinate[3].y = Robot_Position.FOV_Top_Right.y;
    Robot_Position.FOV_corrdinate[4].x = Robot_Position.FOV_Bottom_Right.x;
    Robot_Position.FOV_corrdinate[4].y = Robot_Position.FOV_Bottom_Right.y;
    Robot_Position.FOV_corrdinate[5].x = Robot_Position.FOV_Bottom_Left.x;
    Robot_Position.FOV_corrdinate[5].y = Robot_Position.FOV_Bottom_Left.y;

}

bool ParticleFilter::CheckPointArea(ParticlePoint *tmp, int x, int y, int FOV)
{
    // ROS_INFO("CheckPointArea");
    // ROS_INFO("FOV_Bottom_Right = %d %d",tmp->FOV_Bottom_Left.x,tmp->FOV_Bottom_Right.y); 
    // ROS_INFO("FOV_Top_Right = %d %d",tmp->FOV_Top_Right.x,tmp->FOV_Top_Right.y); 
    // ROS_INFO("FOV_Top_Left = %d %d",tmp->FOV_Top_Left.x,tmp->FOV_Top_Left.y); 
    // ROS_INFO("FOV_Bottom_Left = %d %d",tmp->FOV_Bottom_Left.x,tmp->FOV_Bottom_Left.y);    
    if(x<0 || x>1099 || y < 0 || y>800)
    {
        return false;
    }
    if(FOV == 4)
    {
        int a = (tmp->FOV_Top_Left.x - tmp->FOV_Bottom_Left.x) * (y - tmp->FOV_Bottom_Left.y) - (tmp->FOV_Top_Left.y - tmp->FOV_Bottom_Left.y) * (x - tmp->FOV_Bottom_Left.x);
        int b = (tmp->FOV_Top_Right.x - tmp->FOV_Top_Left.x) * (y - tmp->FOV_Top_Left.y) - (tmp->FOV_Top_Right.y - tmp->FOV_Top_Left.y) * (x - tmp->FOV_Top_Left.x);
        int c = (tmp->FOV_Bottom_Right.x - tmp->FOV_Top_Right.x) * (y - tmp->FOV_Top_Right.y) - (tmp->FOV_Bottom_Right.y - tmp->FOV_Top_Right.y) * (x - tmp->FOV_Top_Right.x);
        int d = (tmp->FOV_Bottom_Left.x - tmp->FOV_Bottom_Right.x) * (y - tmp->FOV_Bottom_Right.y) - (tmp->FOV_Bottom_Left.y - tmp->FOV_Bottom_Right.y) * (x - tmp->FOV_Bottom_Right.x);
        if((a > 0 && b > 0 && c > 0 && d > 0) || (a < 0 && b < 0 && c < 0 && d < 0))
        {
            return true;
        }
        else
        {
            return false;
        }
    }else if(FOV == 5)
    {
        int a = 0;
        int b = 0;
        int c = 0;
        int d = 0;
        int e = 0;
        if(tmp->FOV_tmp.x != -1 && tmp->FOV_tmp.y != -1)
        {
            a = (tmp->FOV_Top_Left.x - tmp->FOV_Bottom_Left.x) * (y - tmp->FOV_Bottom_Left.y) - (tmp->FOV_Top_Left.y - tmp->FOV_Bottom_Left.y) * (x - tmp->FOV_Bottom_Left.x);
            b = (tmp->FOV_tmp.x - tmp->FOV_Top_Left.x) * (y - tmp->FOV_Top_Left.y) - (tmp->FOV_tmp.y - tmp->FOV_Top_Left.y) * (x - tmp->FOV_Top_Left.x);
            c = (tmp->FOV_Top_Right.x - tmp->FOV_tmp.x) * (y - tmp->FOV_tmp.y) - (tmp->FOV_Top_Right.y - tmp->FOV_tmp.y) * (x - tmp->FOV_tmp.x);
            d = (tmp->FOV_Bottom_Right.x - tmp->FOV_Top_Right.x) * (y - tmp->FOV_Top_Right.y) - (tmp->FOV_Bottom_Right.y - tmp->FOV_Top_Right.y) * (x - tmp->FOV_Top_Right.x);
            e = (tmp->FOV_Bottom_Left.x - tmp->FOV_Bottom_Right.x) * (y - tmp->FOV_Bottom_Right.y) - (tmp->FOV_Bottom_Left.y - tmp->FOV_Bottom_Right.y) * (x - tmp->FOV_Bottom_Right.x);
        }else{
            a = (tmp->FOV_Top_Left.x - tmp->FOV_Bottom_Left.x) * (y - tmp->FOV_Bottom_Left.y) - (tmp->FOV_Top_Left.y - tmp->FOV_Bottom_Left.y) * (x - tmp->FOV_Bottom_Left.x);
            b = (tmp->FOV_tmp2.x - tmp->FOV_Top_Left.x) * (y - tmp->FOV_Top_Left.y) - (tmp->FOV_tmp2.y - tmp->FOV_Top_Left.y) * (x - tmp->FOV_Top_Left.x);
            c = (tmp->FOV_Top_Right.x - tmp->FOV_tmp2.x) * (y - tmp->FOV_tmp2.y) - (tmp->FOV_Top_Right.y - tmp->FOV_tmp2.y) * (x - tmp->FOV_tmp2.x);
            d = (tmp->FOV_Bottom_Right.x - tmp->FOV_Top_Right.x) * (y - tmp->FOV_Top_Right.y) - (tmp->FOV_Bottom_Right.y - tmp->FOV_Top_Right.y) * (x - tmp->FOV_Top_Right.x);
            e = (tmp->FOV_Bottom_Left.x - tmp->FOV_Bottom_Right.x) * (y - tmp->FOV_Bottom_Right.y) - (tmp->FOV_Bottom_Left.y - tmp->FOV_Bottom_Right.y) * (x - tmp->FOV_Bottom_Right.x);
        }
        if((a > 0 && b > 0 && c > 0 && d > 0 && e > 0) || (a < 0 && b < 0 && c < 0 && d < 0 && e < 0))
        {
            return true;
        }
        else
        {
            return false;
        }
    }else
    {
        int a = (tmp->FOV_Top_Left.x - tmp->FOV_Bottom_Left.x) * (y - tmp->FOV_Bottom_Left.y) - (tmp->FOV_Top_Left.y - tmp->FOV_Bottom_Left.y) * (x - tmp->FOV_Bottom_Left.x);
        int b = (tmp->FOV_tmp2.x - tmp->FOV_Top_Left.x) * (y - tmp->FOV_Top_Left.y) - (tmp->FOV_tmp2.y - tmp->FOV_Top_Left.y) * (x - tmp->FOV_Top_Left.x);
        int c = (tmp->FOV_tmp.x - tmp->FOV_tmp2.x) * (y - tmp->FOV_tmp2.y) - (tmp->FOV_tmp.y - tmp->FOV_tmp2.y) * (x - tmp->FOV_tmp2.x);
        int d = (tmp->FOV_Top_Right.x - tmp->FOV_tmp.x) * (y - tmp->FOV_tmp.y) - (tmp->FOV_Top_Right.y - tmp->FOV_tmp.y) * (x - tmp->FOV_tmp.x);
        int e = (tmp->FOV_Bottom_Right.x - tmp->FOV_Top_Right.x) * (y - tmp->FOV_Top_Right.y) - (tmp->FOV_Bottom_Right.y - tmp->FOV_Top_Right.y) * (x - tmp->FOV_Top_Right.x);
        int f = (tmp->FOV_Bottom_Left.x - tmp->FOV_Bottom_Right.x) * (y - tmp->FOV_Bottom_Right.y) - (tmp->FOV_Bottom_Left.y - tmp->FOV_Bottom_Right.y) * (x - tmp->FOV_Bottom_Right.x);
        if((a > 0 && b > 0 && c > 0 && d > 0 && e > 0 && f > 0) || (a < 0 && b < 0 && c < 0 && d < 0 && e < 0 && f < 0))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
   
}

void ParticleFilter::FindFeaturePoint()
{
    ROS_INFO("FindFeaturePoint");
    for(int i = 0; i < particlepoint_num; i++)
    {
        if(particlepoint[i].featurepoint_scan_line.size() != 0)
        {
            particlepoint[i].featurepoint_scan_line.clear();
        }
        
        int InnerMsg = 5;
        int OuterMsg = 800;
        int centerx = (particlepoint[i].FOV_Bottom_Right.x + particlepoint[i].FOV_Bottom_Left.x) / 2;
        int centery = (particlepoint[i].FOV_Bottom_Right.y + particlepoint[i].FOV_Bottom_Left.y) / 2;
        int Top_Left_dis = 0;
        int Top_Right_dis = 0;
        int FOV_ = 0;
        if((particlepoint[i].FOV_tmp.x != -1 && particlepoint[i].FOV_tmp.y != -1) && (particlepoint[i].FOV_tmp2.x != -1 && particlepoint[i].FOV_tmp2.y != -1))
        {
            Top_Left_dis = (int)(sqrt(pow((particlepoint[i].FOV_tmp2.x - centerx),2) + pow((particlepoint[i].FOV_tmp2.y - centery),2)));
            Top_Right_dis = (int)(sqrt(pow((particlepoint[i].FOV_tmp.x - centerx),2) + pow((particlepoint[i].FOV_tmp.y - centery),2)));
            FOV_ = 6;
        }else if((particlepoint[i].FOV_tmp.x != -1 && particlepoint[i].FOV_tmp.y != -1) && (particlepoint[i].FOV_tmp2.x == -1 && particlepoint[i].FOV_tmp2.y == -1))
        {
            Top_Left_dis = (int)(sqrt(pow((particlepoint[i].FOV_Top_Left.x - centerx),2) + pow((particlepoint[i].FOV_Top_Left.y - centery),2)));
            Top_Right_dis = (int)(sqrt(pow((particlepoint[i].FOV_tmp.x - centerx),2) + pow((particlepoint[i].FOV_tmp.y - centery),2)));
            FOV_ = 5;
        }else if((particlepoint[i].FOV_tmp.x == -1 && particlepoint[i].FOV_tmp.y == -1) && (particlepoint[i].FOV_tmp2.x != -1 && particlepoint[i].FOV_tmp2.y != -1))
        {
            Top_Left_dis = (int)(sqrt(pow((particlepoint[i].FOV_tmp2.x - centerx),2) + pow((particlepoint[i].FOV_tmp2.y - centery),2)));
            Top_Right_dis = (int)(sqrt(pow((particlepoint[i].FOV_Top_Right.x - centerx),2) + pow((particlepoint[i].FOV_Top_Right.y - centery),2)));
            FOV_ = 5;
        }else{
            Top_Left_dis = (int)(sqrt(pow((particlepoint[i].FOV_Top_Left.x - centerx),2) + pow((particlepoint[i].FOV_Top_Left.y - centery),2)));
            Top_Right_dis = (int)(sqrt(pow((particlepoint[i].FOV_Top_Right.x - centerx),2) + pow((particlepoint[i].FOV_Top_Right.y - centery),2)));
            FOV_ = 4;
        }
        // ROS_INFO("FOV_ = %d",FOV_);
        if(Top_Left_dis > Top_Right_dis)
        {
            OuterMsg = Top_Left_dis + 10;
        }
        else
        {
            OuterMsg = Top_Right_dis + 10;
        }
        // ROS_INFO("OuterMsg = %d",OuterMsg);
        int scan_line_cnt = 0;      //the number of scan_line
        for (float angle = particlepoint[i].FOV_dir +  - 90.0; angle <= particlepoint[i].FOV_dir + 91.0; angle = angle + 5.0)
        {
            bool find_feature_flag = false;
            int angle_be = (int)(normalize_angle(angle));
            scan_line scan_tmp;
            featuredata tmp;

            for (int r = InnerMsg; r <= OuterMsg; r++)
            {      
                int x_ = r * Angle_cos[angle_be];
                int y_ = r * Angle_sin[angle_be];

                int x = centerx + x_;
                int y = centery - y_;
                // ROS_INFO("x = %d y = %d",x,y);

                if(x == 0 || x == (Soccer_Field.cols - 1) || y == 0 || y == (Soccer_Field.rows - 1))
                {
                    scan_line_cnt++;
                    break;
                }
                else
                {
                    // ROS_INFO("CheckPointArea(&particlepoint[i], x, y,FOV_) = %d",CheckPointArea(&particlepoint[i], x, y,FOV_));
                    if(CheckPointArea(&particlepoint[i], x, y,FOV_))
                    { 
                        // ROS_INFO("================");
                        if(Soccer_Field.data[(y * Soccer_Field.cols + x) * 3 + 0] == 255)
                        {
                            if(scan_tmp.feature_point.size() != 0)
                            {
                                int x_dis = x - scan_tmp.feature_point[scan_tmp.feature_point.size() - 1].x;
                                int y_dis = y - scan_tmp.feature_point[scan_tmp.feature_point.size() - 1].y;
                                int p2p_dis = sqrt(pow(x_dis, 2) + pow(y_dis, 2));
                                if(p2p_dis > 10)
                                {
                                    tmp.x = x;
                                    tmp.y = y;
                                    tmp.dis = sqrt(pow((x - particlepoint[i].pos.pose.x),2) + pow((y - particlepoint[i].pos.pose.y),2));
                                    tmp.x_dis = abs(x - particlepoint[i].pos.pose.x);
                                    tmp.y_dis = abs(y - particlepoint[i].pos.pose.y);
                                    // ROS_INFO("scan_tmp.feature_point.size() != 0 tmp = %d %d %d",tmp.x_dis,tmp.y_dis,tmp.dis);
                                    scan_tmp.feature_point.push_back(tmp);
                                }
                            }
                            else
                            {
                                tmp.x = x;
                                tmp.y = y;
                                tmp.dis = sqrt(pow((x - particlepoint[i].pos.pose.x),2) + pow((y - particlepoint[i].pos.pose.y),2));
                                tmp.x_dis = abs(x - particlepoint[i].pos.pose.x);
                                tmp.y_dis = abs(y - particlepoint[i].pos.pose.y);
                                scan_tmp.feature_point.push_back(tmp);
                            }
                            // ROS_INFO("scan_tmp.feature_point.size() == 0 tmp = %d %d %d",tmp.x_dis,tmp.y_dis,tmp.dis);
                            find_feature_flag = true;
                        }
                    }
                    else
                    {
                        scan_line_cnt++;
                        break;
                    }    
                }           
            }
            if(!find_feature_flag)
            {
                featuredata tmp;
                tmp.x = -1;
                tmp.y = -1;
                tmp.dis = -1;
                scan_tmp.feature_point.push_back(tmp);
                // ROS_INFO("!find_feature_flag tmp = %d %d %d",tmp.x,tmp.y,tmp.dis);

            }
            particlepoint[i].featurepoint_scan_line.push_back(scan_tmp);
        }
    }
}

void ParticleFilter::KLD_Sampling()
{
    // ROS_INFO("KLD_Sampling");
    int current_num = 0;
    int particlepoint_num_finish = min_particlepoint_num + 10;
    non_empty_bin.clear();
    //ROS_INFO("particle number is %d !!!!!!!!!!!!!!!!!!!!!!!!",particlepoint_num);

    while(current_num < particlepoint_num && current_num < particlepoint_num_finish)
    {
        Pointdata current_point;
        current_point.pose.x = particlepoint[current_num].pos.pose.x;
        current_point.pose.y = particlepoint[current_num].pos.pose.y;
        current_point.angle = int(particlepoint[current_num].pos.angle);

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
                // ROS_INFO("particlepoint[current_num].pos.pose.x = %d",particlepoint[current_num].pos.pose.x);
                // ROS_INFO("particlepoint[current_num].pos.pose.y = %d",particlepoint[current_num].pos.pose.y);
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
    // ROS_INFO("particle number is %d !!!!!!!!!!!!!!!!!!!!!!!!",particlepoint_num);
    // ROS_INFO("KLD_Sampling end!!");
}


void ParticleFilter::FindRobotPosition(float x_avg, float y_avg)
{
    ROS_INFO("FindRobotPosition");
    // ROS_INFO("x_avg = %f",x_avg);
    // ROS_INFO("y_avg = %f",y_avg);
    float x_varience = 0.0;
    float y_varience = 0.0;
    for(int i = 0; i < particlepoint_num; i++)
    {
        float x_ = particlepoint_compare[i].pos.pose.x - x_avg;
        float y_ = particlepoint_compare[i].pos.pose.y - y_avg;
        x_varience = x_varience + pow(x_, 2);
        y_varience = y_varience + pow(y_, 2);
    }
    x_varience = x_varience / (float)particlepoint_num;
    y_varience = y_varience / (float)particlepoint_num;
    // ROS_INFO("x_varience = %f",x_varience);
    // ROS_INFO("y_varience = %f",y_varience);
    if((x_varience <= 0.5) && (y_varience <= 0.5))
    {
        // ROS_INFO("varience is very small");
        Robot_Position = particlepoint_compare[0];
        return;
    }
    for(int i = 0; i < particlepoint_num; i++)
    {
        float x_to_avg_error = particlepoint_compare[i].pos.pose.x - x_avg;
        float y_to_avg_error = particlepoint_compare[i].pos.pose.y - y_avg;
        x_to_avg_error = pow(x_to_avg_error, 2);
        y_to_avg_error = pow(y_to_avg_error, 2);
        if((x_to_avg_error < x_varience) && (y_to_avg_error < y_varience))
        {
            // ROS_INFO("The best number = %d",i);
            Robot_Position = particlepoint_compare[i];
            break;
        }
        else if(i == (particlepoint_num - 1))
        {
            // ROS_INFO("No find the best");
            Robot_Position.pos.pose.x = x_avg;
            Robot_Position.pos.pose.y = y_avg;
            Robot_Position.pos.angle = particlepoint_compare[0].pos.angle;
        }
    }
}

void ParticleFilter::NoLookField(const movement_data& u)
{
    ROS_INFO("NoLookField");
    if(Step_flag)
    {
        for(int i = 0; i < particlepoint_num; ++i)
        {
            Movement(particlepoint[i],u.straight,u.drift,u.rotational,u.moving,u.dt);          
        }
        Movement(Robot_Position,u.straight,u.drift,u.rotational,u.moving,u.dt); 
        Step_flag = false;
    }
    localization_flag = false;
}


