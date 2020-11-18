#include "ParticleFilter/ParticleFilter.h"

ParticleFilter::ParticleFilter()
{
    rand_angle_init = 5;
    particlepoint_num = PARTICLNUM;
    excellent_particle_num = 20;

    Robot_Position.postion.X = -1; //830  700
    Robot_Position.postion.Y = -1; //640  370
    Robot_Position.angle = -1.0;

    continuous_x = 0;
    continuous_y = 0;

    sendbodyauto_x = 0;
    sendbodyauto_y = 0;

    step_count = 0;

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
    //ROS_INFO("CalcFOVArea");
    for(int i = 0; i < particlepoint_num; i++)
    {
        float FOV_dir = Angle_Adjustment(particlepoint[i].angle + horizontal_head_angle);

        //coordinate Camera_Focus;

        float HFOV_2D_bottom = atan2(bottom_width,bottom) * 180 / PI;
        HFOV_2D_bottom = Angle_Adjustment(HFOV_2D_bottom);
        float HFOV_2D_top = atan2(top_width,top) * 180 / PI;
        HFOV_2D_top = Angle_Adjustment(HFOV_2D_top);

        //Camera_Focus.X = particlepoint[i].postion.X + focus * cos(FOV_dir * DEG2RAD);
        //Camera_Focus.Y = particlepoint[i].postion.Y + focus * sin(FOV_dir * DEG2RAD);

        float right_sight_top_angle = Angle_Adjustment(FOV_dir - HFOV_2D_top);
        float right_sight_bottom_angle = Angle_Adjustment(FOV_dir - HFOV_2D_bottom);
        float left_sight_top_angle = Angle_Adjustment(FOV_dir + HFOV_2D_top);
        float left_sight_bottom_angle = Angle_Adjustment(FOV_dir + HFOV_2D_bottom);
        
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
    float FOV_dir = Angle_Adjustment(Robot_Position.angle + horizontal_head_angle);

    //coordinate Camera_Focus;

    float HFOV_2D_bottom = atan2(bottom_width,bottom) * 180 / PI;
    HFOV_2D_bottom = Angle_Adjustment(HFOV_2D_bottom);
    float HFOV_2D_top = atan2(top_width,top) * 180 / PI;
    HFOV_2D_top = Angle_Adjustment(HFOV_2D_top);

    //Camera_Focus.X = Robot_Position.postion.X + focus * cos(FOV_dir * DEG2RAD);
    //Camera_Focus.Y = Robot_Position.postion.Y + focus * sin(FOV_dir * DEG2RAD);

    float right_sight_top_angle = Angle_Adjustment(FOV_dir - HFOV_2D_top);
    float right_sight_bottom_angle = Angle_Adjustment(FOV_dir - HFOV_2D_bottom);
    float left_sight_top_angle = Angle_Adjustment(FOV_dir + HFOV_2D_top);
    float left_sight_bottom_angle = Angle_Adjustment(FOV_dir + HFOV_2D_bottom);
        
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
        for (float angle = particlepoint[i].angle - 90.0; angle <= particlepoint[i].angle + 91.0; angle = angle + 5.0)
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

                if(x == 0 || x == (Soccer_Filed.cols - 1) || y == 0)
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
        //tmp.clear();
    }
}

void ParticleFilter::FindBestParticle(scan_line *feature_point_observation_data)
{
    ROS_INFO("FindBestParticle");
    for(int i = 0; i < particlepoint_num; i++)
    {
        particlepoint[i].fitness_value = 0;
        particlepoint[i].weight = 0.0;
        int real_feature_point_cnt = 0;
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
    }
    sort(particlepoint.begin(), particlepoint.end(), greater<ParticlePoint>());
    ROS_INFO("weight:%f", particlepoint[0].weight);
    ROS_INFO("fitness:%d", particlepoint[0].fitness_value);
    //ROS_INFO("fitness:%d", particlepoint[1500].fitness_value);
    //-------------判斷適應值太高的狀況就不更新---------
    if(particlepoint[0].weight < 0.5)
    {
        find_best_flag = false;
        AverageRobotPos();
    }
    else
    {
        find_best_flag = true;
        Robot_Position = particlepoint[0];
        //FindRobotPosition();
        ROS_INFO("x:%d", Robot_Position.postion.X);
        ROS_INFO("y:%d", Robot_Position.postion.Y);    
    }
    //-------------判斷適應值太高的狀況就不更新---------
}

void ParticleFilter::FindRobotPosition()
{
    ROS_INFO("FindRobotPosition");
    int x_tmp[particlepoint_num] = {0};
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
    Robot_Position.angle = angle_tmp[particlepoint_num / 2];
}

void ParticleFilter::AverageRobotPos()
{
    vector<ParticlePoint>::iterator f_iter;
    int tmp_x = 0;
    int tmp_y = 0;
    int aa = 0;
    float tmp_angle = 0.0;
    int rand_angle = rand_angle_init * 2 + 1;
    ROS_INFO("continuous_x = %d",continuous_x);
    for (f_iter = particlepoint.begin(); f_iter != particlepoint.end(); ++f_iter)
    {
        aa++;
        f_iter->angle = Angle_Adjustment(Robot_Position.angle + rand()%rand_angle - rand_angle_init); 
        tmp_x += f_iter->postion.X;
        tmp_y += f_iter->postion.Y;
        if(Step_flag)
        {
            float move_x = 0.0;
            float move_y = 0.0;
            float y_dir_angle = 0.0;
            if(continuous_y > 0)
            {
                float y_dir_angle = Angle_Adjustment(f_iter->angle + 90.0);
            }
            else
            {
                float y_dir_angle = Angle_Adjustment(f_iter->angle - 90.0);
            }
            move_x = cos(f_iter->angle * DEG2RAD) * continuous_x / 1000 + cos(y_dir_angle * DEG2RAD) * continuous_y / 1000;
            move_y = -(sin(f_iter->angle * DEG2RAD) * continuous_x / 1000 + sin(y_dir_angle * DEG2RAD) * continuous_y / 1000);
            move_x = 8.5 * move_x;
            move_y = 8.5 * move_y;
            if(aa == 1)
            {
                ROS_INFO("move_x = %f",move_x);
                ROS_INFO("move_y = %f",move_y);
            }
            if(step_count == 0)
            {
                f_iter->postion.X = f_iter->postion.X + (int)(move_x) + (rand() % 11 - 5);
                f_iter->postion.Y = f_iter->postion.Y + (int)(move_y) + (rand() % 11 - 5);
                //ROS_INFO("add the step");
            }
            else
            {
                f_iter->postion.X = f_iter->postion.X + (int)(move_x) * step_count + (rand() % 11 - 5);
                f_iter->postion.Y = f_iter->postion.Y + (int)(move_y) * step_count + (rand() % 11 - 5);
                //ROS_INFO("add the step");
                //step_count = 0;
            }
        }
        else
        {
            f_iter->postion.X = f_iter->postion.X + (int)(cos(f_iter->angle * DEG2RAD) * (rand() % 11 - 5));
            f_iter->postion.Y = f_iter->postion.Y + (int)(sin(f_iter->angle * DEG2RAD) * (rand() % 11 - 5)); 
            //f_iter->postion.X = f_iter->postion.X + (rand() % 31 - 15);
            //f_iter->postion.Y = f_iter->postion.Y + (rand() % 31 - 15);
        }
    }
    ROS_INFO("averageRobotPos");
    Step_flag = false;
    Robot_Position.postion.X = tmp_x / particlepoint_num;        	//粒子數量
    Robot_Position.postion.Y = tmp_y / particlepoint_num;
    Robot_Position.angle = Angle_Adjustment(Robot_Position.angle + rand()%rand_angle - rand_angle_init);
}

void ParticleFilter::CalcNewParticle()
{
    int rand_angle = rand_angle_init * 2 + 1;
    int value_sum = 0;
    float sum = 0.0;
    int particle_num = 0;
    ROS_INFO("calcNewParticle");
    if(excellent_particle_num > particlepoint_num)
    {
        excellent_particle_num = particlepoint_num;
    }

    float proportion[excellent_particle_num] = {0.0};

    for(int i = 0; i < excellent_particle_num; ++i)
    {
        value_sum += particlepoint[i].fitness_value;
    }
    for(int i = 0; i < excellent_particle_num; ++i)
    {
        proportion[i] = (float)(particlepoint[i].fitness_value) / (float)(value_sum);
        sum += proportion[i];
    }
    for(int i = 0; i < excellent_particle_num; ++i)
    {
        particlepoint[i].particle_num = (int)(particlepoint_num * proportion[i] / sum);
        particle_num += particlepoint[i].particle_num;
    }
    particlepoint[0].particle_num += particlepoint_num - particle_num;   //將粒子撒滿
    vector <ParticlePoint> tmp(particlepoint.begin(), particlepoint.end());
    vector <ParticlePoint>::iterator f_iter = tmp.begin();

    particle_num = 0;
    ROS_INFO("continuous_x = %d",continuous_x);
    for (int i = 0; i < excellent_particle_num; ++i)
    {
        for (int j = 0; j < f_iter->particle_num; ++j)
        {
            particlepoint[particle_num].angle = Angle_Adjustment(Robot_Position.angle + rand()%rand_angle - rand_angle_init);
            if(Step_flag)
            {
                float move_x = 0.0;
                float move_y = 0.0;
                float y_dir_angle = 0.0;
                if(continuous_y > 0)
                {
                    float y_dir_angle = Angle_Adjustment(particlepoint[particle_num].angle + 90.0);
                }
                else
                {
                    float y_dir_angle = Angle_Adjustment(particlepoint[particle_num].angle - 90.0);
                }
                move_x = cos(particlepoint[particle_num].angle * DEG2RAD) * continuous_x / 1000 + cos(y_dir_angle * DEG2RAD) * continuous_y / 1000;
                move_y = -(sin(particlepoint[particle_num].angle * DEG2RAD) * continuous_x / 1000 + sin(y_dir_angle * DEG2RAD) * continuous_y / 1000);
                move_x = 7.5 * move_x;
                move_y = 7.5 * move_y;
                if(particle_num == 1)
                {
                    ROS_INFO("move_x = %f",move_x);
                    ROS_INFO("move_y = %f",move_y);
                }
                if(step_count == 0)
                {
                    particlepoint[particle_num].postion.X = f_iter->postion.X + (int)(move_x) + (rand() % 11 - 5);
                    particlepoint[particle_num].postion.Y = f_iter->postion.Y + (int)(move_y) + (rand() % 11 - 5);
                    //ROS_INFO("add the step");
                }
                else
                {
                    particlepoint[particle_num].postion.X = f_iter->postion.X + (int)(move_x) * step_count + (rand() % 11 - 5);
                    particlepoint[particle_num].postion.Y = f_iter->postion.Y + (int)(move_y) * step_count + (rand() % 11 - 5);
                    //ROS_INFO("add the step");
                    //step_count = 0;
                }
            }
            else
            {
                particlepoint[particle_num].postion.X = f_iter->postion.X; //+ (int)(cos(particlepoint[particle_num].angle * DEG2RAD) * (rand() % 11 - 5));
                particlepoint[particle_num].postion.Y = f_iter->postion.Y; //+ (int)(sin(particlepoint[particle_num].angle * DEG2RAD) * (rand() % 11 - 5));
                //particlepoint[particle_num].postion.X = f_iter->postion.X + (rand() % 17 - 8);
                //particlepoint[particle_num].postion.Y = f_iter->postion.Y + (rand() % 17 - 8);
            }
            particle_num++;
        }
        if (f_iter != tmp.end())
        {
            f_iter++;
        }
        else
        {
            break;
        }
    }
    step_count = 0;
    Step_flag = false;
}

void ParticleFilter::NoLookFiled()
{
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
            move_x = 11.5 * move_x;
            move_y = 11.5 * move_y;
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
        move_x = 11.5 * move_x;
        move_y = 11.5 * move_y;
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
        localization_flag = false;
        Step_flag = false;
    }
}