#include "ParticleFilter/ParticleFilter.h"

ParticleFilter::ParticleFilter()
{
    rand_angle_init = 5;
    particlepoint_num = PARTICLNUM;
    excellent_particle_num = EXCELLENTPARTICLNUM;

    Robot_Position.postion.x = -1; //830  700
    Robot_Position.postion.y = -1; //640  370
    Robot_Position.angle = -1.0;

    continuous_x = 0;
    continuous_y = 0;

    sendbodyauto_x = 0;
    sendbodyauto_y = 0;

    step_count = 0;

    //////////////////W_MCL////////////////
    
    factors = { 1, 2, 1, 500, 5,
                1, 2, 1, 500, 7,
                1, 2, 1, 100, 5};
    wfactor = 0;
    rotation = 0;
    regions = {Point(0, 900), Point(0, 600), Point(-180, 180)};
    posx = 0;
    posy = 0;
    rotation = 0;
    total_weight = 0;
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
    use_feature_point = false;
    use_lineinformation = true;
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
    localization_flag = true;
    ParticlePoint tmp;
    R << 0.1, 0, 0, 0.1;
    // noise << 0.005, 0.01, 0.005;
    for (int i = 0; i < particlepoint_num; i++)
    {
        tmp.pos.pose = Point(0,0);
        tmp.pos.angle = 0.0;
        tmp.likehood = 1.0 / particlepoint_num;
        tmp.landmark_list.resize(landmark_size);
        for (int j = 0; j < landmark_size; j++) 
        {
            tmp.landmark_list[j].observed = false;
            tmp.landmark_list[j].mu << 0,0;
            tmp.landmark_list[j].sigma << 0, 0, 0, 0;
        }
        particlepoint.push_back(tmp);
    }
}

void ParticleFilter::Movement(int straight, int drift, int rotational, int moving, int dt)
{
    // ROS_INFO("Movement");    
    if(moving == 1)
    {
        Motion(straight, drift, rotational, moving, dt);
        // ROS_INFO("1");
    }else if(moving == 2)
    {
        GetUpBackUp();
        // ROS_INFO("2");
    }else if(moving == 3){
        GetUpFrontUp();
        // ROS_INFO("3");
    }else{
        Motion(0, 0, 0, 0, dt);
    }
}

void ParticleFilter::Motion(int straight, int drift, int rotational, int moving, int dt)
{
    // ROS_INFO("Motion");    
    int Forward = int(  straight +
                        gauss(factors[0] * straight) +
                        gauss(factors[1] * drift) +
                        gauss(factors[2] * rotational) +
                        gauss(factors[3] * wfactor) +
                        gauss(factors[4]));
    
    int Side  = int(    drift +
                        gauss(factors[5] * straight) +
                        gauss(factors[6] * drift) +
                        gauss(factors[7] * rotational) +
                        gauss(factors[8] * wfactor) +
                        gauss(factors[9]));

    int Omega = int(    rotational +
                        gauss(factors[10] * straight) +
                        gauss(factors[11] * drift) +
                        gauss(factors[12] * rotational) +
                        gauss(factors[13] * wfactor) +
                        gauss(factors[14])); 

    Omega = Omega * DEG2RAD;
    // ROS_INFO("Forward = %d ,Side = %d ,Omega = %d ",Forward,Side,Omega);
    int Theta =  rotation * DEG2RAD;
    int x = 0;
    int y = 0;
    int Direction = 0;
    int Dir2 = 0;
    
    if(Omega == 0)
    {
        Direction = Theta;
        x = (   posx +
                Forward * cos(Theta) * dt +
                Side * sin(Theta) * dt);
        y = (   posy -
                Forward * sin(Theta) * dt +
                Side * cos(Theta) * dt);
    }else{
        Direction = Theta + Omega * dt;
        Dir2 = -Theta + Omega * dt;
        x = (posx +
            Forward / Omega * (sin(Direction) - sin(Theta)) -
            Side / Omega * (cos(-Theta) - cos(Dir2)));
        y = (posy -
            Forward / Omega * (cos(Theta) - cos(Direction)) -
            Side / Omega * (sin(-Theta) - sin(Dir2)));
    }

    if( x < regions[0].x || x > regions[0].y ||
        y < regions[1].x || y > regions[1].y )
    {
        x = 0;
        y = 0;
    }

    posx = x;
    posy = y;
    float rotat = Direction * RAD2DEG;
    rotation = normalize_angle(rotat);
    // ROS_INFO("posx = %d ,posy = %d ,rotation = %d ",posx,posy,rotation);
}

void ParticleFilter::GetUpBackUp()
{
    // ROS_INFO("GetUpBackUp");    
    posx += int(gauss(7, 0));
    posy += int(gauss(7, 0));
    rotation += int(gauss(25, 0));
}

void ParticleFilter::GetUpFrontUp()
{   
    // ROS_INFO("GetUpFrontUp");    
    posx += int(gauss(7,-30)*sin(rotation*DEG2RAD));
    posy += int(gauss(7,-30)*cos(rotation*DEG2RAD));
    rotation += int(gauss(25, 0));
    GetUpBackUp();
}

double ParticleFilter::calcWeight(scan_line *feature_point_observation_data, LineINF *Line_observation_data)
{
    if(use_feature_point)
    {
        for(int i = 0; i < particlepoint_num; i++)
        {
            particlepoint[i].fitness_value = 0;
            particlepoint[i].weight = 0.0;
            int real_feature_point_cnt = 0;
            for(int j = 0; j < particlepoint[i].featurepoint_scan_line.size(); j++)//36
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
            particlepoint[i].weight = (float)particlepoint[i].fitness_value / ((float)(real_feature_point_cnt * MAP_MAX_LENGTH))+(float)particlepoint[i].likehood;
        }
    }
    if(use_lineinformation)
    {
        for(int i = 0; i < particlepoint_num; i++)
        {
            vector<double> P;
            particlepoint[i].weight = 0.0;
            for(int m = 0; m < particlepoint[i].landmark_list.size(); m++)//計算虛擬地圖中的地標相似性
            {
                Pointdata robot = {Point(0,0),0};
                robot.pose = particlepoint[i].pos.pose;
                robot.angle = particlepoint[i].pos.angle;
                if((m -1) >= 0)
                {
                    double gx = robot.pose.x + particlepoint[i].landmark_list[m].distance * cos(robot.angle + particlepoint[i].landmark_list[m].Line_theta);
                    double gy = robot.pose.y + particlepoint[i].landmark_list[m].distance * sin(robot.angle + particlepoint[i].landmark_list[m].Line_theta);
                    particlepoint[i].landmark_list[m-1].mu << gx, gy;
                    //get the Jacobian with respect to the landmark position
                    Eigen::MatrixXd G;
                    Eigen::Vector2d g;
                    Measurement_model(particlepoint[i], m, g, G);
                    
                    //initialize the ekf for this landmark
                    MatrixXd Gi = G.inverse();
                    particlepoint[i].landmark_list[m-1].sigma = Gi * R * Gi.transpose();

                    Eigen::Vector2d expect_z;
                    Eigen::MatrixXd G;
                    Measurement_model(particlepoint[i],m, expect_z, G) ;
                    
                    //compute the measurement covariance
                    Eigen::MatrixXd sig = particlepoint[i].landmark_list[m - 1].sigma;
                    Eigen::MatrixXd Z   = G * sig * G.transpose() + R;
                    //calculate the Kalman gain K
                    Eigen::MatrixXd K = sig * G.transpose() * Z.inverse();
                    //calculat the error between the z and expected Z
                    Eigen::Vector2d z_actual;
                    if(Line_observation_data_flag == true)
                    {
                        if(m < Line_observation_data_Size-1)
                        {
                            ROS_INFO("(*(Line_observation_data + m)).distance = %f",(*(Line_observation_data + m)).distance);
                            ROS_INFO("(*(Line_observation_data + m)).Line_theta = %f",(*(Line_observation_data + m)).Line_theta);
                            z_actual << (*(Line_observation_data + m)).distance, (*(Line_observation_data + m)).Line_theta;
                        }else{
                            z_actual << 0.0, 0.0;
                        }
                    }else{
                        z_actual << 0.0, 0.0;
                    }
                    Eigen::Vector2d z_diff = z_actual - expect_z;
                    z_diff(1) = normalize_angle(z_diff(1));
                    // cout<<" sig "<< endl << sig <<endl;
                    // cout<<" Z " << endl <<  Z <<endl;
                    // cout<<" K " << endl <<  K <<endl;            
                    particlepoint[i].landmark_list[m-1].mu    = particlepoint[i].landmark_list[m-1].mu + K * z_diff;
                    particlepoint[i].landmark_list[m-1].sigma = particlepoint[i].landmark_list[m-1].sigma - K * G * sig;
                    ROS_INFO("particlepoint[%d].landmark_list[m-1].mu = %f",i,particlepoint[i].landmark_list[m-1].mu);    
                    ROS_INFO("particlepoint[%d].landmark_list[m-1].sigma = %f",i,particlepoint[i].landmark_list[m-1].sigma); 
                    //calculate the weight
                    double p = exp(-0.5*z_diff.transpose()*Z.inverse()*z_diff)/sqrt(2 * PI * Z.determinant());
                    // ROS_INFO("p = %f, exp(-0.5*z_diff.transpose()*Z.inverse()*z_diff) = %f,sqrt(2 * PI * Z.determinant()) = %f",p,exp(-0.5*z_diff.transpose()*Z.inverse()*z_diff),sqrt(2 * PI * Z.determinant()));    
                    P.push_back(p);
                    // particlepoint[i].likehood *= p;
                    // ROS_INFO("particlepoint[%d].likehood = %f",i,particlepoint[i].likehood);      
                }
            }
            double maxP = *max_element(P.begin(),P.end()); 
            particlepoint[i].weight = maxP;
        }
    }
    int totalweight = 0;
    totalweight += particlepoint[i].weight;
}

int ParticleFilter::TournamentResample(int excellent_particle_num)
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

void ParticleFilter::GetBestPoseAndLandmark(VectorXd& mu_ )
{
    int index = TournamentResample(excellent_particle_num);
    ParticlePoint& current_particle = particlepoint[index];
    int N_ = current_particle.landmark_list.size();
      
    mu_ = Eigen::VectorXd::Zero(2 * N_ + 3);
    mu_(0) = current_particle.pos.angle;
    mu_(1) = current_particle.pos.pose.x;
    mu_(2) = current_particle.pos.pose.y;

    for (int i = 0; i < N_; i++) {
        mu_(2 * i + 3) = current_particle.landmark_list[i].mu(0);
        mu_(2 * i + 4) = current_particle.landmark_list[i].mu(1);
    }
}

// void ParticleFilter::ParticlePointInitialize(unsigned int landmark_size)
// {
//     ROS_INFO("ParticlePointinit");
//     localization_flag = true;
//     // time_t t;
//     // srand((unsigned) time(&t));
//     ParticlePoint tmp;
//     // int rand_angle = rand_angle_init * 2 + 1;        //粒子隨機角度//大數   rand()%40 - 20 = -20 ~ 20
//     R << 0.1, 0, 0, 0.1;
//     for (int i = 0; i < particlepoint_num; i++)
//     {
//         //原本
//         // tmp.angle = normalize_angle(Robot_Position.angle + rand()%rand_angle - rand_angle_init);
//         // tmp.postion.x = Robot_Position.postion.x + (rand() % 31 - 15);   //-3 ~ 3
//         // tmp.postion.y = Robot_Position.postion.y + (rand() % 31 - 15);
        
//         tmp.pos.pose = Point(0,0);
//         tmp.pos.angle = 0.0;
//         tmp.likehood = 1.0 / particlepoint_num;
//         tmp.landmark_list.resize(landmark_size);
//         for (int j = 0; j < landmark_size; j++) 
//         {
//             tmp.landmark_list[j].observed = false;
//             tmp.landmark_list[j].mu << 0,0;
//             tmp.landmark_list[j].sigma << 0, 0, 0, 0;
//         }
//         //tmp.angle = 135.0;
//         //tmp.postion.x = 520;   //-3 ~ 3
//         //tmp.postion.y = 370;
//         particlepoint.push_back(tmp);
//     }
// }
void ParticleFilter::CalcFOVArea(int focus, int top, int bottom, int top_width, int bottom_width, float horizontal_head_angle)
{
    ROS_INFO("CalcFOVArea");
    for(int i = 0; i < particlepoint_num; i++)
    {
        particlepoint[i].FOV_dir = normalize_angle(particlepoint[i].angle + horizontal_head_angle);

        //coordinate Camera_Focus;

        float HFOV_2D_bottom = atan2(bottom_width,bottom) * 180 / PI;
        HFOV_2D_bottom = normalize_angle(HFOV_2D_bottom);
        float HFOV_2D_top = atan2(top_width,top) * 180 / PI;
        HFOV_2D_top = normalize_angle(HFOV_2D_top);

        //Camera_Focus.x = particlepoint[i].postion.x + focus * cos(FOV_dir * DEG2RAD);
        //Camera_Focus.y = particlepoint[i].postion.y + focus * sin(FOV_dir * DEG2RAD);

        float right_sight_top_angle = normalize_angle(particlepoint[i].FOV_dir - HFOV_2D_top);
        float right_sight_bottom_angle = normalize_angle(particlepoint[i].FOV_dir - HFOV_2D_bottom);
        float left_sight_top_angle = normalize_angle(particlepoint[i].FOV_dir + HFOV_2D_top);
        float left_sight_bottom_angle = normalize_angle(particlepoint[i].FOV_dir + HFOV_2D_bottom);
        
        float top_waist_length = sqrt(pow(top,2) + pow(top_width,2));
        float bottom_waist_length = sqrt(pow(bottom,2) + pow(bottom_width,2));

        
        
        particlepoint[i].FOV_Top_Right.x = particlepoint[i].postion.x + top_waist_length * cos(right_sight_top_angle * DEG2RAD);
        particlepoint[i].FOV_Top_Right.x = Frame_Area(particlepoint[i].FOV_Top_Right.x, MAP_LENGTH);
        particlepoint[i].FOV_Top_Right.y = particlepoint[i].postion.y - top_waist_length * sin(right_sight_top_angle * DEG2RAD);   
        particlepoint[i].FOV_Top_Right.y = Frame_Area(particlepoint[i].FOV_Top_Right.y, MAP_WIDTH);
        particlepoint[i].FOV_Bottom_Right.x = particlepoint[i].postion.x + bottom_waist_length * cos(right_sight_bottom_angle * DEG2RAD);
        particlepoint[i].FOV_Bottom_Right.x = Frame_Area(particlepoint[i].FOV_Bottom_Right.x, MAP_LENGTH);
        particlepoint[i].FOV_Bottom_Right.y = particlepoint[i].postion.y - bottom_waist_length * sin(right_sight_bottom_angle * DEG2RAD);
        particlepoint[i].FOV_Bottom_Right.y = Frame_Area(particlepoint[i].FOV_Bottom_Right.y, MAP_WIDTH);

        particlepoint[i].FOV_Top_Left.x = particlepoint[i].postion.x + top_waist_length * cos(left_sight_top_angle * DEG2RAD);
        particlepoint[i].FOV_Top_Left.x = Frame_Area(particlepoint[i].FOV_Top_Left.x, MAP_LENGTH);
        particlepoint[i].FOV_Top_Left.y = particlepoint[i].postion.y - top_waist_length * sin(left_sight_top_angle * DEG2RAD);
        particlepoint[i].FOV_Top_Left.y = Frame_Area(particlepoint[i].FOV_Top_Left.y, MAP_WIDTH);
        particlepoint[i].FOV_Bottom_Left.x = particlepoint[i].postion.x + bottom_waist_length * cos(left_sight_bottom_angle * DEG2RAD);
        particlepoint[i].FOV_Bottom_Left.x = Frame_Area(particlepoint[i].FOV_Bottom_Left.x, MAP_LENGTH);
        particlepoint[i].FOV_Bottom_Left.y = particlepoint[i].postion.y - bottom_waist_length * sin(left_sight_bottom_angle * DEG2RAD); 
        particlepoint[i].FOV_Bottom_Left.y = Frame_Area(particlepoint[i].FOV_Bottom_Left.y, MAP_WIDTH); 
        
        particlepoint[i].FOV_corrdinate[0].x = particlepoint[i].FOV_Top_Left.x;
        particlepoint[i].FOV_corrdinate[0].y = particlepoint[i].FOV_Top_Left.y;
        particlepoint[i].FOV_corrdinate[1].x = particlepoint[i].FOV_Top_Right.x;
        particlepoint[i].FOV_corrdinate[1].y = particlepoint[i].FOV_Top_Right.y;
        particlepoint[i].FOV_corrdinate[2].x = particlepoint[i].FOV_Bottom_Right.x;
        particlepoint[i].FOV_corrdinate[2].y = particlepoint[i].FOV_Bottom_Right.y;
        particlepoint[i].FOV_corrdinate[3].x = particlepoint[i].FOV_Bottom_Left.x;
        particlepoint[i].FOV_corrdinate[3].y = particlepoint[i].FOV_Bottom_Left.y;
        
    }
}

void ParticleFilter::FindLandMarkInVirtualField(ParticlePoint *particlepoint)
{
    // ROS_INFO("FindLandMarkInVirtualField");
    FieldLine_data FOV_function_data_tmp[4]; // To caculate FOV's linear function
    particlepoint->landmark_list.clear();
    
    for(int j = 0; j < 4; j++)
    {
        Point FOV_tmp = particlepoint->FOV_corrdinate[j];
        Point FOV_tmp2 = particlepoint->FOV_corrdinate[0];
        Point FOV_tmp1 = Point(0,0);
        if(j<3)
        {
            FOV_tmp1 = particlepoint->FOV_corrdinate[j+1];
        }
        int x1 = 0;
        int y1 = 0;
        int x2 = 0;
        int y2 = 0;
        if(j<3)
        {
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
    }
    // ROS_INFO("FOV_function_data_tmp[0].start_point = %d,%d FOV_function_data_tmp[0].end_point = %d,%d",
                // FOV_function_data_tmp[0].start_point.x,FOV_function_data_tmp[0].start_point.y,
                // FOV_function_data_tmp[0].end_point.x,FOV_function_data_tmp[0].end_point.y);
    vector<Point> intersect_point_list;
    vector<Point> virtualpointinFOV;
    vector<Vec4i> detect_line_list;
    intersect_point_list.clear();
    virtualpointinFOV.clear();
    detect_line_list.clear();  
    for(int k = 0; k < field_list.size()-1; k++) // find the line in FOV (virtual field)
    {
        Vec4i X = {field_list[k].start_point.x,field_list[k].start_point.y,field_list[k].end_point.x,field_list[k].end_point.y};
        int startpointinarea = CheckPointArea(particlepoint, X[0], X[1]);
        int endpointinarea = CheckPointArea(particlepoint, X[2], X[3]);
        if(startpointinarea == 1)
        {
            // ROS_INFO("startpointinarea == 1");
            virtualpointinFOV.push_back(Point(X[0], X[1]));
        }else if(endpointinarea == 1)
        {
            // ROS_INFO("endpointinarea == 1");
            virtualpointinFOV.push_back(Point(X[2], X[3]));
        }
        int count;
        for(int l = 0; l < 4; l++)
        {
            Vec4i Y = {FOV_function_data_tmp[l].start_point.x,FOV_function_data_tmp[l].start_point.y,FOV_function_data_tmp[l].end_point.x,FOV_function_data_tmp[l].end_point.y};
            count = 0;
            if(intersect(X,Y) == 1)//相交
            {
                Point intersectpoint = IntersectPoint(X,Y);
                intersect_point_list.push_back(intersectpoint);
                // ROS_INFO("intersectpoint = %d,%d",intersectpoint.x,intersectpoint.y);
            }else if(intersect(X,Y) == 2)//重合
            {
                Vec4i tmp = {0,0,0,0};
                Vec4i tmp1 = {FOV_function_data_tmp[2].start_point.x,FOV_function_data_tmp[2].start_point.y,FOV_function_data_tmp[2].end_point.x,FOV_function_data_tmp[2].end_point.y};
                if(MinDistance(tmp1,Point(Y[0],Y[1])) < MinDistance(tmp1,Point(Y[2],Y[3])))
                {
                    tmp = {Y[0],Y[1],Y[2],Y[3]};
                    detect_line_list.push_back(tmp);
                    // ROS_INFO("%d %d %d %d",tmp[0],tmp[1],tmp[2],tmp[3]);
                }else{
                    tmp = {Y[2],Y[3],Y[1],Y[2]};
                    detect_line_list.push_back(tmp);
                    // ROS_INFO("%d %d %d %d",tmp[0],tmp[1],tmp[2],tmp[3]);
                }
            }else{
                count++;
            }  
        }
        if(count == 4)
        {
            break;
        }
        if(virtualpointinFOV.size() == 0) //FOV中無末端點
        {
            if(intersect_point_list.size() == 2)
            {
                Vec4i tmp = {0,0,0,0};
                Vec4i tmp1 = {FOV_function_data_tmp[2].start_point.x,FOV_function_data_tmp[2].start_point.y,FOV_function_data_tmp[2].end_point.x,FOV_function_data_tmp[2].end_point.y};
                if(MinDistance(tmp1,intersect_point_list[0]) < MinDistance(tmp1,intersect_point_list[1]))
                {
                    tmp = {intersect_point_list[0].x,intersect_point_list[0].y,intersect_point_list[1].x,intersect_point_list[1].y};
                    detect_line_list.push_back(tmp);
                    intersect_point_list.clear();
                    // ROS_INFO("%d %d %d %d",tmp[0],tmp[1],tmp[2],tmp[3]);
                }else{
                    tmp = {intersect_point_list[1].x,intersect_point_list[1].y,intersect_point_list[0].x,intersect_point_list[0].y};
                    detect_line_list.push_back(tmp);
                    intersect_point_list.clear();
                    // ROS_INFO("%d %d %d %d",tmp[0],tmp[1],tmp[2],tmp[3]);
                }
            }else{
                intersect_point_list.clear();
            }
        }else if(virtualpointinFOV.size() != 0 && intersect_point_list.size() != 0) //FOV中有末端點且至少有一交點
        {
            Vec4i tmp = {0,0,0,0};
            Vec4i tmp1 = {FOV_function_data_tmp[2].start_point.x,FOV_function_data_tmp[2].start_point.y,FOV_function_data_tmp[2].end_point.x,FOV_function_data_tmp[2].end_point.y};
            if(MinDistance(tmp1,virtualpointinFOV[0]) < MinDistance(tmp1,intersect_point_list[0]))
            {
                tmp = {virtualpointinFOV[0].x,virtualpointinFOV[0].y,intersect_point_list[0].x,intersect_point_list[0].y};
                detect_line_list.push_back(tmp);
                intersect_point_list.clear();
                virtualpointinFOV.clear();
                // ROS_INFO("%d %d %d %d",tmp[0],tmp[1],tmp[2],tmp[3]);
            }else{
                tmp = {intersect_point_list[0].x,intersect_point_list[0].y,virtualpointinFOV[0].x,virtualpointinFOV[0].y};
                detect_line_list.push_back(tmp);
                intersect_point_list.clear();
                virtualpointinFOV.clear();
                // ROS_INFO("%d %d %d %d",tmp[0],tmp[1],tmp[2],tmp[3]);
            }
        }
    }
    sort(detect_line_list.begin(), detect_line_list.end(), tocompare);
    // ROS_INFO("detect_line_list");
    // for(int n = 0; n < detect_line_list.size(); n++)
    // {
    //     Vec4i tmp = detect_line_list[n];      
    //     ROS_INFO("%d %d %d %d",tmp[0],tmp[1],tmp[2],tmp[3]);
    // }
    for(int m = 0; m < detect_line_list.size(); m++)
    {
        LineINF landmark_tmp;
        Vec4i tmp = detect_line_list[m];
        landmark_tmp = LineInformation(Point(tmp[0],tmp[1]),Point(tmp[2],tmp[3]),FOV_function_data_tmp[2].start_point,FOV_function_data_tmp[2].end_point);
        particlepoint->landmark_list.push_back(landmark_tmp);
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
    Robot_Position.FOV_dir = normalize_angle(Robot_Position.angle + horizontal_head_angle);

    //coordinate Camera_Focus;

    float HFOV_2D_bottom = atan2(bottom_width,bottom) * 180 / PI;
    HFOV_2D_bottom = normalize_angle(HFOV_2D_bottom);
    float HFOV_2D_top = atan2(top_width,top) * 180 / PI;
    HFOV_2D_top = normalize_angle(HFOV_2D_top);

    //Camera_Focus.x = Robot_Position.postion.x + focus * cos(FOV_dir * DEG2RAD);
    //Camera_Focus.y = Robot_Position.postion.y + focus * sin(FOV_dir * DEG2RAD);

    float right_sight_top_angle = normalize_angle(Robot_Position.FOV_dir - HFOV_2D_top);
    float right_sight_bottom_angle = normalize_angle(Robot_Position.FOV_dir - HFOV_2D_bottom);
    float left_sight_top_angle = normalize_angle(Robot_Position.FOV_dir + HFOV_2D_top);
    float left_sight_bottom_angle = normalize_angle(Robot_Position.FOV_dir + HFOV_2D_bottom);
        
    float top_waist_length = sqrt(pow(top,2) + pow(top_width,2));
    float bottom_waist_length = sqrt(pow(bottom,2) + pow(bottom_width,2));

    Robot_Position.FOV_Top_Right.x = Robot_Position.postion.x + top_waist_length * cos(right_sight_top_angle * DEG2RAD);
    Robot_Position.FOV_Top_Right.x = Frame_Area(Robot_Position.FOV_Top_Right.x, MAP_LENGTH);
    Robot_Position.FOV_Top_Right.y = Robot_Position.postion.y - top_waist_length * sin(right_sight_top_angle * DEG2RAD);   
    Robot_Position.FOV_Top_Right.y = Frame_Area(Robot_Position.FOV_Top_Right.y, MAP_WIDTH);
    Robot_Position.FOV_Bottom_Right.x = Robot_Position.postion.x + bottom_waist_length * cos(right_sight_bottom_angle * DEG2RAD);
    Robot_Position.FOV_Bottom_Right.x = Frame_Area(Robot_Position.FOV_Bottom_Right.x, MAP_LENGTH);
    Robot_Position.FOV_Bottom_Right.y = Robot_Position.postion.y - bottom_waist_length * sin(right_sight_bottom_angle * DEG2RAD);
    Robot_Position.FOV_Bottom_Right.y = Frame_Area(Robot_Position.FOV_Bottom_Right.y, MAP_WIDTH);

    Robot_Position.FOV_Top_Left.x = Robot_Position.postion.x + top_waist_length * cos(left_sight_top_angle * DEG2RAD);
    Robot_Position.FOV_Top_Left.x = Frame_Area(Robot_Position.FOV_Top_Left.x, MAP_LENGTH);
    Robot_Position.FOV_Top_Left.y = Robot_Position.postion.y - top_waist_length * sin(left_sight_top_angle * DEG2RAD);
    Robot_Position.FOV_Top_Left.y = Frame_Area(Robot_Position.FOV_Top_Left.y, MAP_WIDTH);
    Robot_Position.FOV_Bottom_Left.x = Robot_Position.postion.x + bottom_waist_length * cos(left_sight_bottom_angle * DEG2RAD);
    Robot_Position.FOV_Bottom_Left.x = Frame_Area(Robot_Position.FOV_Bottom_Left.x, MAP_LENGTH);
    Robot_Position.FOV_Bottom_Left.y = Robot_Position.postion.y - bottom_waist_length * sin(left_sight_bottom_angle * DEG2RAD); 
    Robot_Position.FOV_Bottom_Left.y = Frame_Area(Robot_Position.FOV_Bottom_Left.y, MAP_WIDTH); 

    Robot_Position.FOV_corrdinate[0].x = Robot_Position.FOV_Top_Left.x;
    Robot_Position.FOV_corrdinate[0].y = Robot_Position.FOV_Top_Left.y;
    Robot_Position.FOV_corrdinate[1].x = Robot_Position.FOV_Top_Right.x;
    Robot_Position.FOV_corrdinate[1].y = Robot_Position.FOV_Top_Right.y;
    Robot_Position.FOV_corrdinate[2].x = Robot_Position.FOV_Bottom_Right.x;
    Robot_Position.FOV_corrdinate[2].y = Robot_Position.FOV_Bottom_Right.y;
    Robot_Position.FOV_corrdinate[3].x = Robot_Position.FOV_Bottom_Left.x;
    Robot_Position.FOV_corrdinate[3].y = Robot_Position.FOV_Bottom_Left.y;
}

bool ParticleFilter::CheckPointArea(ParticlePoint *tmp, int x, int y)
{
    // ROS_INFO("CheckPointArea");
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
        int Top_Left_dis = (int)(sqrt(pow((particlepoint[i].FOV_Top_Left.x - centerx),2) + pow((particlepoint[i].FOV_Top_Left.y - centery),2)));
        int Top_Right_dis = (int)(sqrt(pow((particlepoint[i].FOV_Top_Right.y - centerx),2) + pow((particlepoint[i].FOV_Top_Right.y - centery),2)));
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
            int angle_be = (int)(normalize_angle(angle));
            scan_line scan_tmp;
            for (int r = InnerMsg; r <= OuterMsg; r++)
            {      
                int x_ = r * Angle_cos[angle_be];
                int y_ = r * Angle_sin[angle_be];
                int x = Frame_Area(centerx + x_, Soccer_Field.cols);
                int y = Frame_Area(centery - y_, Soccer_Field.rows);
                if(x == 0 || x == (Soccer_Field.cols - 1) || y == 0 || y == (Soccer_Field.rows - 1))
                {
                    if(!find_feature_flag)
                    {
                        featuredata tmp;
                        tmp.x = -1;
                        tmp.y = -1;
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
                        if(Soccer_Field.data[(y * Soccer_Field.cols + x) * 3 + 0] == 255)
                        {
                            featuredata tmp;
                            if(scan_tmp.feature_point.size() != 0)
                            {
                                int x_dis = x - scan_tmp.feature_point[scan_tmp.feature_point.size() - 1].x;
                                int y_dis = y - scan_tmp.feature_point[scan_tmp.feature_point.size() - 1].y;
                                int p2p_dis = sqrt(pow(x_dis, 2) + pow(y_dis, 2));
                                if(p2p_dis > 10)
                                {
                                    tmp.x = x;
                                    tmp.y = y;
                                    tmp.dis = sqrt(pow((x - particlepoint[i].postion.x),2) + pow((y - particlepoint[i].postion.y),2));
                                    tmp.x_dis = abs(x - particlepoint[i].postion.x);
                                    tmp.y_dis = abs(y - particlepoint[i].postion.y);
                                    scan_tmp.feature_point.push_back(tmp);
                                }
                            }
                            else
                            {
                                tmp.x = x;
                                tmp.y = y;
                                tmp.dis = sqrt(pow((x - particlepoint[i].postion.x),2) + pow((y - particlepoint[i].postion.y),2));
                                tmp.x_dis = abs(x - particlepoint[i].postion.x);
                                tmp.y_dis = abs(y - particlepoint[i].postion.y);
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
                            tmp.x = -1;
                            tmp.y = -1;
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

void ParticleFilter::KLD_Sampling()
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
                    /*if(current_point.pose.y != non_empty_bin[i].pose.y)
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



void ParticleFilter::FindRobotPosition(float x_avg, float y_avg)
{
    ROS_INFO("FindRobotPosition");
    ROS_INFO("x_avg = %f",x_avg);
    ROS_INFO("y_avg = %f",y_avg);
    float x_varience = 0.0;
    float y_varience = 0.0;
    for(int i = 0; i < particlepoint_num; i++)
    {
        float x_ = particlepoint_compare[i].postion.x - x_avg;
        float y_ = particlepoint_compare[i].postion.y - y_avg;
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
        float x_to_avg_error = particlepoint_compare[i].postion.x - x_avg;
        float y_to_avg_error = particlepoint_compare[i].postion.y - y_avg;
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
            Robot_Position.postion.x = x_avg;
            Robot_Position.postion.y = y_avg;
            Robot_Position.angle = particlepoint_compare[0].angle;
        }
    }
    /*int x_tmp[particlepoint_num] = {0};
    int y_tmp[particlepoint_num] = {0};
    float angle_tmp[particlepoint_num] = {0.0};
    for(int i = 0; i < particlepoint_num; i++)
    {
        x_tmp[i] = particlepoint[i].postion.x;
        y_tmp[i] = particlepoint[i].postion.y;
        angle_tmp[i] = particlepoint[i].angle;
    }
    sort(x_tmp, x_tmp + particlepoint_num);
    sort(y_tmp, y_tmp + particlepoint_num);
    sort(angle_tmp,angle_tmp + particlepoint_num);
    Robot_Position.postion.x = x_tmp[particlepoint_num / 2];
    Robot_Position.postion.y = y_tmp[particlepoint_num / 2];
    Robot_Position.angle = angle_tmp[particlepoint_num / 2];*/
}







// void ParticleFilter::StatePredict(vector<int> u)
// {
    // ROS_INFO("StatePredict");

    //--------------orignal----------
    // int rand_angle = rand_angle_init * 2 + 1;
    // int value_sum = 0;
    // float sum = 0.0;
    // int particle_num = 0;

    // if(excellent_particle_num > particlepoint_num)
    // {
    //     excellent_particle_num = particlepoint_num;
    // }
    // ///////////////////////////Augmented_MCL ///////////////////////////
    // std::random_device rd, x_rd, y_rd;                                          //產生亂數種子                    //Produce the random seed
    // std::mt19937 generator(rd());                                               //使用梅森旋轉演算法產生亂數        //Use Mersenne Twister to produce the random
    // std::mt19937 x_generator(x_rd());
    // std::mt19937 y_generator(y_rd());
    // std::uniform_real_distribution<float> add_random_distribution(0.0, 1.0);    //設定機率分佈範圍（連續型均勻分佈） //Set the random distribution range(continuous)
    // std::uniform_real_distribution<float> x_random_distribution(550, MAP_LENGTH - 105), y_random_distribution(10, MAP_WIDTH - 10);
    // float reset_random_threshold = std::max(0.0, 1.0 - (weight_fast / weight_slow));
    // int rand_particle_cnt = 0;
    // ROS_INFO("reset_random_threshold = %f",reset_random_threshold);
    ///////////////////////////Augmented_MCL ///////////////////////////
    // for(int i = 0; i < particlepoint_num; ++i)
    // { 
    //     double random = ((double)rand() / (RAND_MAX));
    //     ParticlePoint current_particle;
    //     if(random < reset_random_threshold)
    //     {
    //         ROS_INFO("Kidnapped");
    //         find_best_flag = true;
    //         current_particle.postion.x = x_random_distribution(x_generator);
    //         current_particle.postion.y = y_random_distribution(y_generator);
    //         current_particle.angle = normalize_angle(Robot_Position.angle + rand()%rand_angle - rand_angle_init);
    //         tmp.push_back(current_particle);
    //     }
    //     else
    //     {
    //         int best_particle_num = TournamentResample(excellent_particle_num);

    //         current_particle.angle = normalize_angle(Robot_Position.angle + rand()%rand_angle - rand_angle_init);
    //         if(Step_flag)
    //         {
    //             float move_x = 0.0;
    //             float move_y = 0.0;
    //             float y_dir_angle = 0.0;
    //             if(continuous_y > 0)
    //             {
    //                 float y_dir_angle = normalize_angle(current_particle.angle + 90.0);
    //             }
    //             else
    //             {
    //                 float y_dir_angle = normalize_angle(current_particle.angle - 90.0);
    //             }
    //             move_x = cos(current_particle.angle * DEG2RAD) * continuous_x / 1000 + cos(y_dir_angle * DEG2RAD) * continuous_y / 1000;
    //             move_y = -(sin(current_particle.angle * DEG2RAD) * continuous_x / 1000 + sin(y_dir_angle * DEG2RAD) * continuous_y / 1000);
    //             move_x = 6.5 * move_x;
    //             move_y = 6.5 * move_y;
    //             /*if(i == 1)
    //             {
    //                 ROS_INFO("move_x = %f",move_x);
    //                 ROS_INFO("move_y = %f",move_y);
    //             }*/
    //             if(step_count == 0)
    //             {
    //                 current_particle.postion.x = particlepoint[best_particle_num].postion.x + (int)(move_x) + (rand() % 11 - 5);
    //                 current_particle.postion.y = particlepoint[best_particle_num].postion.y + (int)(move_y) + (rand() % 11 - 5);
    //             }
    //             else
    //             {
    //                 current_particle.postion.x = particlepoint[best_particle_num].postion.x + (int)(move_x) * step_count + (rand() % 11 - 5);
    //                 current_particle.postion.y = particlepoint[best_particle_num].postion.y + (int)(move_y) * step_count + (rand() % 11 - 5);
    //             }
    //         }
    //         else
    //         {
    //             current_particle.postion.x = particlepoint[best_particle_num].postion.x; //+ (rand() % 11 - 5);
    //             current_particle.postion.y = particlepoint[best_particle_num].postion.y; //+ (rand() % 11 - 5);
    //             //particlepoint[particle_num].postion.x = f_iter->postion.x + (rand() % 17 - 8);
    //             //particlepoint[particle_num].postion.y = f_iter->postion.y + (rand() % 17 - 8);
    //         }
    //         tmp.push_back(current_particle);
    //     }
    // }
    // particlepoint.clear();
    // particlepoint = tmp;
    // ROS_INFO("particlepoint size = %d",particlepoint.size());
    // step_count = 0;
    // Step_flag = false;
// }

void ParticleFilter::NoLookField(vector<int> u)
{
    ROS_INFO("NoLookField");
    if(Step_flag)
    {
        for(int i = 0; i < particlepoint_num; ++i)
        {
            posx = particlepoint[i].postion.x;
            posy = particlepoint[i].postion.y;
            rotation = particlepoint[i].angle;
            Movement(u[0],u[1],u[2],u[3],u[4]);          
            particlepoint[i].postion.x = posx;
            particlepoint[i].postion.y = posy;
            particlepoint[i].angle = rotation;
            ROS_INFO("posx = %d,posy = %d,rotation = %d",posx,posy,rotation);
        }
        posx = Robot_Position.postion.x;
        posy = Robot_Position.postion.y;
        rotation = Robot_Position.angle;
        Movement(u[0],u[1],u[2],u[3],u[4]); 
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
        Step_flag = false;
    }
    localization_flag = false;
}

