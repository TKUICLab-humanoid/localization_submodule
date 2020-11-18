#include "ParticleFilter/ParticleFilter.h"

ParticleFilter::ParticleFilter()
{
    rand_angle_init = 5;
    particlepoint_num = PARTICLNUM;
    excellent_particle_num = EXCELLENTPARTICLNUM;

    Robot_Position.postion.X = -1; //830  700
    Robot_Position.postion.Y = -1; //640  370
    Robot_Position.angle = -1.0;

    continuous_x = 0;
    continuous_y = 0;

    sendbodyauto_x = 0;
    sendbodyauto_y = 0;

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
    alpha_slow = 0.003;//0.062;
    alpha_fast = 0.1;//0.1;
    //////////////////Augmented_MCL//////////////////

    localization_flag = true;
    find_best_flag = true;

    AngleLUT();
}
ParticleFilter::~ParticleFilter()
{
    Angle_sin.clear();
    Angle_cos.clear();
}

void ParticleFilter::ParticlePointinit()
{
    ROS_INFO("ParticlePointinit");
    localization_flag = true;
    time_t t;
    srand((unsigned) time(&t));
    ParticlePoint tmp;
    int rand_angle = rand_angle_init * 2 + 1;        //粒子隨機角度//大數   rand()%40 - 20 = -20 ~ 20
    for (int i = 0; i < particlepoint_num; i++)
    {
        tmp.angle = Angle_Adjustment(Robot_Position.angle + rand()%rand_angle - rand_angle_init);
        tmp.postion.X = Robot_Position.postion.X + (rand() % 31 - 15);   //-3 ~ 3
        tmp.postion.Y = Robot_Position.postion.Y + (rand() % 31 - 15);
        //tmp.angle = 135.0;
        //tmp.postion.X = 520;   //-3 ~ 3
        //tmp.postion.Y = 370;
        particlepoint.push_back(tmp);
    }
}

void ParticleFilter::CalcFOVArea(int focus, int top, int bottom, int top_width, int bottom_width, float horizontal_head_angle)
{
    ROS_INFO("CalcFOVArea");
    for(int i = 0; i < particlepoint_num; i++)
    {
        particlepoint[i].FOV_dir = Angle_Adjustment(particlepoint[i].angle + horizontal_head_angle);

        //coordinate Camera_Focus;

        float HFOV_2D_bottom = atan2(bottom_width,bottom) * 180 / PI;
        HFOV_2D_bottom = Angle_Adjustment(HFOV_2D_bottom);
        float HFOV_2D_top = atan2(top_width,top) * 180 / PI;
        HFOV_2D_top = Angle_Adjustment(HFOV_2D_top);

        //Camera_Focus.X = particlepoint[i].postion.X + focus * cos(FOV_dir * DEG2RAD);
        //Camera_Focus.Y = particlepoint[i].postion.Y + focus * sin(FOV_dir * DEG2RAD);

        float right_sight_top_angle = Angle_Adjustment(particlepoint[i].FOV_dir - HFOV_2D_top);
        float right_sight_bottom_angle = Angle_Adjustment(particlepoint[i].FOV_dir - HFOV_2D_bottom);
        float left_sight_top_angle = Angle_Adjustment(particlepoint[i].FOV_dir + HFOV_2D_top);
        float left_sight_bottom_angle = Angle_Adjustment(particlepoint[i].FOV_dir + HFOV_2D_bottom);
        
        float top_waist_length = sqrt(pow(top,2) + pow(top_width,2));
        float bottom_waist_length = sqrt(pow(bottom,2) + pow(bottom_width,2));

        particlepoint[i].FOV_Top_Right.X = particlepoint[i].postion.X + top_waist_length * cos(right_sight_top_angle * DEG2RAD);
        particlepoint[i].FOV_Top_Right.X = Frame_Area(particlepoint[i].FOV_Top_Right.X, MAP_LENGTH);
        particlepoint[i].FOV_Top_Right.Y = particlepoint[i].postion.Y - top_waist_length * sin(right_sight_top_angle * DEG2RAD);   
        particlepoint[i].FOV_Top_Right.Y = Frame_Area(particlepoint[i].FOV_Top_Right.Y, MAP_WIDTH);
        particlepoint[i].FOV_Bottom_Right.X = particlepoint[i].postion.X + bottom_waist_length * cos(right_sight_bottom_angle * DEG2RAD);
        particlepoint[i].FOV_Bottom_Right.X = Frame_Area(particlepoint[i].FOV_Bottom_Right.X, MAP_LENGTH);
        particlepoint[i].FOV_Bottom_Right.Y = particlepoint[i].postion.Y - bottom_waist_length * sin(right_sight_bottom_angle * DEG2RAD);
        particlepoint[i].FOV_Bottom_Right.Y = Frame_Area(particlepoint[i].FOV_Bottom_Right.Y, MAP_WIDTH);

        particlepoint[i].FOV_Top_Left.X = particlepoint[i].postion.X + top_waist_length * cos(left_sight_top_angle * DEG2RAD);
        particlepoint[i].FOV_Top_Left.X = Frame_Area(particlepoint[i].FOV_Top_Left.X, MAP_LENGTH);
        particlepoint[i].FOV_Top_Left.Y = particlepoint[i].postion.Y - top_waist_length * sin(left_sight_top_angle * DEG2RAD);
        particlepoint[i].FOV_Top_Left.Y = Frame_Area(particlepoint[i].FOV_Top_Left.Y, MAP_WIDTH);
        particlepoint[i].FOV_Bottom_Left.X = particlepoint[i].postion.X + bottom_waist_length * cos(left_sight_bottom_angle * DEG2RAD);
        particlepoint[i].FOV_Bottom_Left.X = Frame_Area(particlepoint[i].FOV_Bottom_Left.X, MAP_LENGTH);
        particlepoint[i].FOV_Bottom_Left.Y = particlepoint[i].postion.Y - bottom_waist_length * sin(left_sight_bottom_angle * DEG2RAD); 
        particlepoint[i].FOV_Bottom_Left.Y = Frame_Area(particlepoint[i].FOV_Bottom_Left.Y, MAP_WIDTH); 
    }
}

void ParticleFilter::CalcFOVArea_averagepos(int focus, int top, int bottom, int top_width, int bottom_width, float horizontal_head_angle)
{
    Robot_Position.FOV_dir = Angle_Adjustment(Robot_Position.angle + horizontal_head_angle);

    //coordinate Camera_Focus;

    float HFOV_2D_bottom = atan2(bottom_width,bottom) * 180 / PI;
    HFOV_2D_bottom = Angle_Adjustment(HFOV_2D_bottom);
    float HFOV_2D_top = atan2(top_width,top) * 180 / PI;
    HFOV_2D_top = Angle_Adjustment(HFOV_2D_top);

    //Camera_Focus.X = Robot_Position.postion.X + focus * cos(FOV_dir * DEG2RAD);
    //Camera_Focus.Y = Robot_Position.postion.Y + focus * sin(FOV_dir * DEG2RAD);

    float right_sight_top_angle = Angle_Adjustment(Robot_Position.FOV_dir - HFOV_2D_top);
    float right_sight_bottom_angle = Angle_Adjustment(Robot_Position.FOV_dir - HFOV_2D_bottom);
    float left_sight_top_angle = Angle_Adjustment(Robot_Position.FOV_dir + HFOV_2D_top);
    float left_sight_bottom_angle = Angle_Adjustment(Robot_Position.FOV_dir + HFOV_2D_bottom);
        
    float top_waist_length = sqrt(pow(top,2) + pow(top_width,2));
    float bottom_waist_length = sqrt(pow(bottom,2) + pow(bottom_width,2));

    Robot_Position.FOV_Top_Right.X = Robot_Position.postion.X + top_waist_length * cos(right_sight_top_angle * DEG2RAD);
    Robot_Position.FOV_Top_Right.X = Frame_Area(Robot_Position.FOV_Top_Right.X, MAP_LENGTH);
    Robot_Position.FOV_Top_Right.Y = Robot_Position.postion.Y - top_waist_length * sin(right_sight_top_angle * DEG2RAD);   
    Robot_Position.FOV_Top_Right.Y = Frame_Area(Robot_Position.FOV_Top_Right.Y, MAP_WIDTH);
    Robot_Position.FOV_Bottom_Right.X = Robot_Position.postion.X + bottom_waist_length * cos(right_sight_bottom_angle * DEG2RAD);
    Robot_Position.FOV_Bottom_Right.X = Frame_Area(Robot_Position.FOV_Bottom_Right.X, MAP_LENGTH);
    Robot_Position.FOV_Bottom_Right.Y = Robot_Position.postion.Y - bottom_waist_length * sin(right_sight_bottom_angle * DEG2RAD);
    Robot_Position.FOV_Bottom_Right.Y = Frame_Area(Robot_Position.FOV_Bottom_Right.Y, MAP_WIDTH);

    Robot_Position.FOV_Top_Left.X = Robot_Position.postion.X + top_waist_length * cos(left_sight_top_angle * DEG2RAD);
    Robot_Position.FOV_Top_Left.X = Frame_Area(Robot_Position.FOV_Top_Left.X, MAP_LENGTH);
    Robot_Position.FOV_Top_Left.Y = Robot_Position.postion.Y - top_waist_length * sin(left_sight_top_angle * DEG2RAD);
    Robot_Position.FOV_Top_Left.Y = Frame_Area(Robot_Position.FOV_Top_Left.Y, MAP_WIDTH);
    Robot_Position.FOV_Bottom_Left.X = Robot_Position.postion.X + bottom_waist_length * cos(left_sight_bottom_angle * DEG2RAD);
    Robot_Position.FOV_Bottom_Left.X = Frame_Area(Robot_Position.FOV_Bottom_Left.X, MAP_LENGTH);
    Robot_Position.FOV_Bottom_Left.Y = Robot_Position.postion.Y - bottom_waist_length * sin(left_sight_bottom_angle * DEG2RAD); 
    Robot_Position.FOV_Bottom_Left.Y = Frame_Area(Robot_Position.FOV_Bottom_Left.Y, MAP_WIDTH); 
}

bool ParticleFilter::CheckPointArea(ParticlePoint *tmp, int x, int y)
{
    int a = (tmp->FOV_Top_Left.X - tmp->FOV_Bottom_Left.X) * (y - tmp->FOV_Bottom_Left.Y) - (tmp->FOV_Top_Left.Y - tmp->FOV_Bottom_Left.Y) * (x - tmp->FOV_Bottom_Left.X);
    int b = (tmp->FOV_Top_Right.X - tmp->FOV_Top_Left.X) * (y - tmp->FOV_Top_Left.Y) - (tmp->FOV_Top_Right.Y - tmp->FOV_Top_Left.Y) * (x - tmp->FOV_Top_Left.X);
    int c = (tmp->FOV_Bottom_Right.X - tmp->FOV_Top_Right.X) * (y - tmp->FOV_Top_Right.Y) - (tmp->FOV_Bottom_Right.Y - tmp->FOV_Top_Right.Y) * (x - tmp->FOV_Top_Right.X);
    int d = (tmp->FOV_Bottom_Left.X - tmp->FOV_Bottom_Right.X) * (y - tmp->FOV_Bottom_Right.Y) - (tmp->FOV_Bottom_Left.Y - tmp->FOV_Bottom_Right.Y) * (x - tmp->FOV_Bottom_Right.X);

    if((a > 0 && b > 0 && c > 0 && d > 0) || (a < 0 && b < 0 && c < 0 && d < 0))
    {
        return true;
    }
    else
    {
        return false;
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
        int centerx = (particlepoint[i].FOV_Bottom_Right.X + particlepoint[i].FOV_Bottom_Left.X) / 2;
        int centery = (particlepoint[i].FOV_Bottom_Right.Y + particlepoint[i].FOV_Bottom_Left.Y) / 2;
        int Top_Left_dis = (int)(sqrt(pow((particlepoint[i].FOV_Top_Left.X - centerx),2) + pow((particlepoint[i].FOV_Top_Left.Y - centery),2)));
        int Top_Right_dis = (int)(sqrt(pow((particlepoint[i].FOV_Top_Right.X - centerx),2) + pow((particlepoint[i].FOV_Top_Right.Y - centery),2)));
        if(Top_Left_dis > Top_Right_dis)
        {
            OuterMsg = Top_Left_dis + 10;
        }
        else
        {
            OuterMsg = Top_Right_dis + 10;
        }
        int scan_line_cnt = 0;      //the number of scan_line
        for (float angle = particlepoint[i].FOV_dir +  - 90.0; angle <= particlepoint[i].FOV_dir + 91.0; angle = angle + 5.0)
        {
            bool find_feature_flag = false;
            int angle_be = (int)(Angle_Adjustment(angle));
            scan_line scan_tmp;
            for (int r = InnerMsg; r <= OuterMsg; r++)
            {      
                int x_ = r * Angle_cos[angle_be];
                int y_ = r * Angle_sin[angle_be];
                int x = Frame_Area(centerx + x_, Soccer_Filed.cols);
                int y = Frame_Area(centery - y_, Soccer_Filed.rows);

                if(x == 0 || x == (Soccer_Filed.cols - 1) || y == 0 || y == (Soccer_Filed.rows - 1))
                {
                    if(!find_feature_flag)
                    {
                        featuredata tmp;
                        tmp.X = -1;
                        tmp.Y = -1;
                        tmp.dis = -1;
                        scan_tmp.feature_point.push_back(tmp);
                    }
                    particlepoint[i].featurepoint_scan_line.push_back(scan_tmp);
                    scan_line_cnt++;
                    break;
                }
                else
                {
                    if(CheckPointArea(&particlepoint[i], x, y))
                    { 
                        if(Soccer_Filed.data[(y * Soccer_Filed.cols + x) * 3 + 0] == 255)
                        {
                            featuredata tmp;
                            if(scan_tmp.feature_point.size() != 0)
                            {
                                int x_dis = x - scan_tmp.feature_point[scan_tmp.feature_point.size() - 1].X;
                                int y_dis = y - scan_tmp.feature_point[scan_tmp.feature_point.size() - 1].Y;
                                int p2p_dis = sqrt(pow(x_dis, 2) + pow(y_dis, 2));
                                if(p2p_dis > 10)
                                {
                                    tmp.X = x;
                                    tmp.Y = y;
                                    tmp.dis = sqrt(pow((x - particlepoint[i].postion.X),2) + pow((y - particlepoint[i].postion.Y),2));
                                    tmp.x_dis = abs(x - particlepoint[i].postion.X);
                                    tmp.y_dis = abs(y - particlepoint[i].postion.Y);
                                    scan_tmp.feature_point.push_back(tmp);
                                }
                            }
                            else
                            {
                                tmp.X = x;
                                tmp.Y = y;
                                tmp.dis = sqrt(pow((x - particlepoint[i].postion.X),2) + pow((y - particlepoint[i].postion.Y),2));
                                tmp.x_dis = abs(x - particlepoint[i].postion.X);
                                tmp.y_dis = abs(y - particlepoint[i].postion.Y);
                                scan_tmp.feature_point.push_back(tmp);
                            }
                            find_feature_flag = true;
                        }
                    }
                    else
                    {
                        if(!find_feature_flag)
                        {
                            featuredata tmp;
                            tmp.X = -1;
                            tmp.Y = -1;
                            tmp.dis = -1;
                            scan_tmp.feature_point.push_back(tmp);
                        }
                        particlepoint[i].featurepoint_scan_line.push_back(scan_tmp);
                        scan_line_cnt++;
                        break;
                    }    
                }           
            }
        }
    }
}

void ParticleFilter::FindBestParticle(scan_line *feature_point_observation_data)
{
    ROS_INFO("FindBestParticle");
    float x_avg = 0.0;
    float y_avg = 0.0;
    for(int i = 0; i < particlepoint_num; i++)
    {
        particlepoint[i].fitness_value = 0;
        particlepoint[i].weight = 0.0;
        int real_feature_point_cnt = 0;
        x_avg += particlepoint[i].postion.X;
        y_avg += particlepoint[i].postion.Y;
        for(int j = 0; j < particlepoint[i].featurepoint_scan_line.size(); j++)
        {
            real_feature_point_cnt += (*(feature_point_observation_data + j)).feature_point.size();
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
                particlepoint[i].fitness_value += (scan_line_fitness / particlepoint[i].featurepoint_scan_line[j].feature_point.size());
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
                particlepoint[i].fitness_value += (scan_line_fitness / particlepoint[i].featurepoint_scan_line[j].feature_point.size());
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
        }
        particlepoint[i].weight = (float)particlepoint[i].fitness_value / (float)(real_feature_point_cnt * MAP_MAX_LENGTH);
        weight_avg = weight_avg + particlepoint[i].weight;
    }
    x_avg = x_avg / (float)particlepoint_num;
    y_avg = y_avg / (float)particlepoint_num;
    weight_avg = weight_avg / (float)particlepoint_num;
    ROS_INFO("weight_avg:%f", weight_avg);
    weight_slow = weight_slow + alpha_slow * (weight_avg - weight_slow);
    weight_fast = weight_fast + alpha_fast * (weight_avg - weight_fast);
    ROS_INFO("weight_slow:%f", weight_slow);
    ROS_INFO("weight_fast:%f", weight_fast);
    particlepoint_compare = particlepoint;
    sort(particlepoint_compare.begin(), particlepoint_compare.end(), greater<ParticlePoint>());
    FindRobotPosition(x_avg, y_avg);
    ROS_INFO("weight:%f", particlepoint[0].weight);
    ROS_INFO("weight_low:%f", particlepoint[particlepoint.size() - 1].weight);
    ROS_INFO("x:%d", Robot_Position.postion.X);
    ROS_INFO("y:%d", Robot_Position.postion.Y);
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

void ParticleFilter::FindRobotPosition(float x_avg, float y_avg)
{
    ROS_INFO("FindRobotPosition");
    ROS_INFO("x_avg = %f",x_avg);
    ROS_INFO("y_avg = %f",y_avg);
    float x_varience = 0.0;
    float y_varience = 0.0;
    for(int i = 0; i < particlepoint_num; i++)
    {
        float x_ = particlepoint_compare[i].postion.X - x_avg;
        float y_ = particlepoint_compare[i].postion.Y - y_avg;
        x_varience = x_varience + pow(x_, 2);
        y_varience = y_varience + pow(y_, 2);
    }
    x_varience = x_varience / (float)particlepoint_num;
    y_varience = y_varience / (float)particlepoint_num;
    ROS_INFO("x_varience = %f",x_varience);
    ROS_INFO("y_varience = %f",y_varience);
    if((x_varience <= 0.5) && (y_varience <= 0.5))
    {
        ROS_INFO("varience is very small");
        Robot_Position = particlepoint_compare[0];
        return;
    }
    for(int i = 0; i < particlepoint_num; i++)
    {
        float x_to_avg_error = particlepoint_compare[i].postion.X - x_avg;
        float y_to_avg_error = particlepoint_compare[i].postion.Y - y_avg;
        x_to_avg_error = pow(x_to_avg_error, 2);
        y_to_avg_error = pow(y_to_avg_error, 2);
        if((x_to_avg_error < x_varience) && (y_to_avg_error < y_varience))
        {
            ROS_INFO("The best number = %d",i);
            Robot_Position = particlepoint_compare[i];
            break;
        }
        else if(i == (particlepoint_num - 1))
        {
            ROS_INFO("No find the best");
            Robot_Position.postion.X = x_avg;
            Robot_Position.postion.Y = y_avg;
            Robot_Position.angle = particlepoint_compare[0].angle;
        }
    }
    /*int x_tmp[particlepoint_num] = {0};
    int y_tmp[particlepoint_num] = {0};
    float angle_tmp[particlepoint_num] = {0.0};
    for(int i = 0; i < particlepoint_num; i++)
    {
        x_tmp[i] = particlepoint[i].postion.X;
        y_tmp[i] = particlepoint[i].postion.Y;
        angle_tmp[i] = particlepoint[i].angle;
    }
    sort(x_tmp, x_tmp + particlepoint_num);
    sort(y_tmp, y_tmp + particlepoint_num);
    sort(angle_tmp,angle_tmp + particlepoint_num);
    Robot_Position.postion.X = x_tmp[particlepoint_num / 2];
    Robot_Position.postion.Y = y_tmp[particlepoint_num / 2];
    Robot_Position.angle = angle_tmp[particlepoint_num / 2];*/
}

int ParticleFilter::TournamentResample(int excellent_particle_num)
{
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

void ParticleFilter::StatePredict()
{
    ROS_INFO("StatePredict");
    int rand_angle = rand_angle_init * 2 + 1;
    int value_sum = 0;
    float sum = 0.0;
    int particle_num = 0;

    if(excellent_particle_num > particlepoint_num)
    {
        excellent_particle_num = particlepoint_num;
    }
    ///////////////////////////Augmented_MCL ///////////////////////////
    std::random_device rd, x_rd, y_rd;                                          //產生亂數種子                    //Produce the random seed
    std::mt19937 generator(rd());                                               //使用梅森旋轉演算法產生亂數        //Use Mersenne Twister to produce the random
    std::mt19937 x_generator(x_rd());
    std::mt19937 y_generator(y_rd());
    std::uniform_real_distribution<float> add_random_distribution(0.0, 1.0);    //設定機率分佈範圍（連續型均勻分佈） //Set the random distribution range(continuous)
    std::uniform_real_distribution<float> x_random_distribution(550, MAP_LENGTH - 105), y_random_distribution(10, MAP_WIDTH - 10);
    float reset_random_threshold = std::max(0.0, 1.0 - (weight_fast / weight_slow));
    int rand_particle_cnt = 0;
    ROS_INFO("reset_random_threshold = %f",reset_random_threshold);
    ///////////////////////////Augmented_MCL ///////////////////////////
    vector<ParticlePoint> tmp;

    //ROS_INFO("continuous_x = %d",continuous_x);
    for(int i = 0; i < particlepoint_num; ++i)
    { 
        double random = ((double)rand() / (RAND_MAX));
        ParticlePoint current_particle;
        if(random < reset_random_threshold)
        {
            ROS_INFO("Kidnapped");
            find_best_flag = true;
            current_particle.postion.X = x_random_distribution(x_generator);
            current_particle.postion.Y = y_random_distribution(y_generator);
            current_particle.angle = Angle_Adjustment(Robot_Position.angle + rand()%rand_angle - rand_angle_init);
            tmp.push_back(current_particle);
        }
        else
        {
            int best_particle_num = TournamentResample(excellent_particle_num);

            current_particle.angle = Angle_Adjustment(Robot_Position.angle + rand()%rand_angle - rand_angle_init);
            if(Step_flag)
            {
                float move_x = 0.0;
                float move_y = 0.0;
                float y_dir_angle = 0.0;
                if(continuous_y > 0)
                {
                    float y_dir_angle = Angle_Adjustment(current_particle.angle + 90.0);
                }
                else
                {
                    float y_dir_angle = Angle_Adjustment(current_particle.angle - 90.0);
                }
                move_x = cos(current_particle.angle * DEG2RAD) * continuous_x / 1000 + cos(y_dir_angle * DEG2RAD) * continuous_y / 1000;
                move_y = -(sin(current_particle.angle * DEG2RAD) * continuous_x / 1000 + sin(y_dir_angle * DEG2RAD) * continuous_y / 1000);
                move_x = 6.5 * move_x;
                move_y = 6.5 * move_y;
                /*if(i == 1)
                {
                    ROS_INFO("move_x = %f",move_x);
                    ROS_INFO("move_y = %f",move_y);
                }*/
                if(step_count == 0)
                {
                    current_particle.postion.X = particlepoint[best_particle_num].postion.X + (int)(move_x) + (rand() % 11 - 5);
                    current_particle.postion.Y = particlepoint[best_particle_num].postion.Y + (int)(move_y) + (rand() % 11 - 5);
                }
                else
                {
                    current_particle.postion.X = particlepoint[best_particle_num].postion.X + (int)(move_x) * step_count + (rand() % 11 - 5);
                    current_particle.postion.Y = particlepoint[best_particle_num].postion.Y + (int)(move_y) * step_count + (rand() % 11 - 5);
                }
            }
            else
            {
                current_particle.postion.X = particlepoint[best_particle_num].postion.X; //+ (rand() % 11 - 5);
                current_particle.postion.Y = particlepoint[best_particle_num].postion.Y; //+ (rand() % 11 - 5);
                //particlepoint[particle_num].postion.X = f_iter->postion.X + (rand() % 17 - 8);
                //particlepoint[particle_num].postion.Y = f_iter->postion.Y + (rand() % 17 - 8);
            }
            tmp.push_back(current_particle);
        }
    }
    particlepoint.clear();
    particlepoint = tmp;
    ROS_INFO("particlepoint size = %d",particlepoint.size());
    step_count = 0;
    Step_flag = false;
}

void ParticleFilter::NoLookFiled()
{
    ROS_INFO("NoLookFiled");
    if(Step_flag)
    {
        float move_x = 0.0;
        float move_y = 0.0;
        float y_dir_angle = 0.0;
        for(int i = 0; i < particlepoint_num; ++i)
        { 
            particlepoint[i].angle = Angle_Adjustment(Robot_Position.angle);
            if(continuous_y > 0)
            {
                float y_dir_angle = Angle_Adjustment(particlepoint[i].angle + 90.0);
            }
            else
            {
                float y_dir_angle = Angle_Adjustment(particlepoint[i].angle - 90.0);
            }
            move_x = cos(particlepoint[i].angle * DEG2RAD) * continuous_x / 1000 + cos(y_dir_angle * DEG2RAD) * continuous_y / 1000;
            move_y = -(sin(particlepoint[i].angle * DEG2RAD) * continuous_x / 1000 + sin(y_dir_angle * DEG2RAD) * continuous_y / 1000);
            move_x = 6.5 * move_x;
            move_y = 6.5 * move_y;
            if(step_count == 0)
            {
                particlepoint[i].postion.X = particlepoint[i].postion.X + (int)(move_x);
                particlepoint[i].postion.Y = particlepoint[i].postion.Y + (int)(move_y);
                //ROS_INFO("add the step");
            }
            else
            {
                particlepoint[i].postion.X = particlepoint[i].postion.X + (int)(move_x) * step_count;
                particlepoint[i].postion.Y = particlepoint[i].postion.Y + (int)(move_y) * step_count;
                //ROS_INFO("add the step");
                //step_count = 0;
            }
        }
        Robot_Position.angle = Angle_Adjustment(Robot_Position.angle);
        if(continuous_y > 0)
        {
            float y_dir_angle = Angle_Adjustment(Robot_Position.angle + 90.0);
        }
        else
        {
            float y_dir_angle = Angle_Adjustment(Robot_Position.angle - 90.0);
        }
        move_x = cos(Robot_Position.angle * DEG2RAD) * continuous_x / 1000 + cos(y_dir_angle * DEG2RAD) * continuous_y / 1000;
        move_y = -(sin(Robot_Position.angle * DEG2RAD) * continuous_x / 1000 + sin(y_dir_angle * DEG2RAD) * continuous_y / 1000);
        move_x = 6.5 * move_x;
        move_y = 6.5 * move_y;
        if(step_count == 0)
        {
            Robot_Position.postion.X = Robot_Position.postion.X + (int)(move_x);
            Robot_Position.postion.Y = Robot_Position.postion.Y + (int)(move_y);
            //ROS_INFO("add the step");
        }
        else
        {
            Robot_Position.postion.X = Robot_Position.postion.X + (int)(move_x) * step_count;
            Robot_Position.postion.Y = Robot_Position.postion.Y + (int)(move_y) * step_count;
            //ROS_INFO("add the step");
            //step_count = 0;
        }
        Step_flag = false;
    }
    localization_flag = false;
}

void ParticleFilter::KLD_Sampling()
{
    ROS_INFO("KLD_Sampling");
    int current_num = 0;
    int particlepoint_num_finish = min_particlepoint_num + 10;
    non_empty_bin.clear();
    //ROS_INFO("particle number is %d !!!!!!!!!!!!!!!!!!!!!!!!",particlepoint_num);

    while(current_num < particlepoint_num && current_num < particlepoint_num_finish)
    {
        pointdata current_point;
        current_point.pos.X = particlepoint[current_num].postion.X;
        current_point.pos.Y = particlepoint[current_num].postion.Y;
        current_point.theta = int(particlepoint[current_num].angle);

        if(non_empty_bin.size() != 0)
        {
            for(int i = 0; i < non_empty_bin.size(); i++)
            {
                int dis_x = abs(current_point.pos.X - non_empty_bin[i].pos.X);
                int dis_y = abs(current_point.pos.Y - non_empty_bin[i].pos.Y);
                int dis = sqrt(pow(dis_x,2) + pow(dis_y,2));
                //ROS_INFO("dis = %d",dis);
                // ROS_INFO("current_point.pos.X = %d",current_point.pos.X);
                // ROS_INFO("current_point.pos.Y = %d",current_point.pos.Y);
                // ROS_INFO("particlepoint[current_num].postion.X = %d",particlepoint[current_num].postion.X);
                // ROS_INFO("particlepoint[current_num].postion.Y = %d",particlepoint[current_num].postion.Y);
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
                    /*if(current_point.pos.Y != non_empty_bin[i].pos.Y)
                    {
                        if(current_point.theta != non_empty_bin[i].theta)
                        {
                            non_empty_bin.push_back(current_point);
                            if(current_num >= min_particlepoint_num)
                            {
                                ROS_INFO("non_empty_bin.size = %d",non_empty_bin.size());
                                float kld_equation_1 = float((non_empty_bin.size() - 1)) / (2.0 * kld_err);
                                //ROS_INFO("kld_equation_1 is %f !!!!!!!!!!!!!!!!!!!!!!!!",kld_equation_1);
                                float kld_equation_2 = 1.0 - 2.0 / (9.0 * float((non_empty_bin.size() - 1)));
                                //ROS_INFO("kld_equation_2 is %f !!!!!!!!!!!!!!!!!!!!!!!!",kld_equation_2);
                                float kld_equation_3 = sqrt(2.0 / (9.0 * (non_empty_bin.size() - 1))) * kld_z;
                                //ROS_INFO("kld_equation_3 is %f !!!!!!!!!!!!!!!!!!!!!!!!",kld_equation_3);
                                particlepoint_num = int(kld_equation_1 * pow((kld_equation_2 + kld_equation_3),3));
                            }
                            break;
                        }
                    }*/
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