#include "Particle/Particle.h"

Particle::Particle()
{
    rotation = 0.0;
    posx = 0;
    posy = 0;
    weight = 0.0;
    //////////////////W_MCL////////////////test///
    factors = { 1, 2, 1, 500, 5,
                1, 2, 1, 500, 7,
                1, 2, 1, 100, 5};
    wfactor = 0;
    SigmaIMU = 10;
    R << 0.1, 0, 0, 0.1;
    particlepoint_num = PARTICLNUM;
}

Particle::~Particle()
{

}

void Particle::Movement(float straight, float drift, float rotational, int moving, float dt)
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

void Particle::Motion(float straight, float drift, float rotational, int moving, float dt)
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
  
    posx = x;
    posy = y;
    float rotat = Direction * RAD2DEG;
    rotation = normalize_angle(rotat);
    // ROS_INFO("posx = %d ,posy = %d ,rotation = %d ",posx,posy,rotation);
}

void Particle::GetUpBackUp()
{
    // ROS_INFO("GetUpBackUp");    
    posx += int(gauss(7, 0));
    posy += int(gauss(7, 0));
    rotation += int(gauss(25, 0));
}

void Particle::GetUpFrontUp()
{   
    // ROS_INFO("GetUpFrontUp");    
    posx += int(gauss(7,-30)*sin(rotation*DEG2RAD));
    posy += int(gauss(7,-30)*cos(rotation*DEG2RAD));
    rotation += int(gauss(25, 0));
    GetUpBackUp();
}

void Particle::CalcFOVArea(int focus, int top, int bottom, int top_width, int bottom_width, float horizontal_head_angle)
{
    ROS_INFO("CalcFOVArea");
    for(auto& p : particles)
    {
        p.FOV_dir = normalize_angle(p.angle + horizontal_head_angle);

        //coordinate Camera_Focus;
        float HFOV_2D_bottom = atan2(bottom_width,bottom) * 180 / PI;
        HFOV_2D_bottom = normalize_angle(HFOV_2D_bottom);
        float HFOV_2D_top = atan2(top_width,top) * 180 / PI;
        HFOV_2D_top = normalize_angle(HFOV_2D_top);

        //Camera_Focus.x = p.postion.x + focus * cos(FOV_dir * DEG2RAD);
        //Camera_Focus.y = p.postion.y + focus * sin(FOV_dir * DEG2RAD);

        float right_sight_top_angle = normalize_angle(p.FOV_dir - HFOV_2D_top);
        float right_sight_bottom_angle = normalize_angle(p.FOV_dir - HFOV_2D_bottom);
        float left_sight_top_angle = normalize_angle(p.FOV_dir + HFOV_2D_top);
        float left_sight_bottom_angle = normalize_angle(p.FOV_dir + HFOV_2D_bottom);
        
        float top_waist_length = sqrt(pow(top,2) + pow(top_width,2));
        float bottom_waist_length = sqrt(pow(bottom,2) + pow(bottom_width,2));
        
        p.FOV_Top_Right.x = p.postion.x + top_waist_length * cos(right_sight_top_angle * DEG2RAD);
        p.FOV_Top_Right.x = Frame_Area(p.FOV_Top_Right.x, MAP_LENGTH);
        p.FOV_Top_Right.y = p.postion.y - top_waist_length * sin(right_sight_top_angle * DEG2RAD);   
        p.FOV_Top_Right.y = Frame_Area(p.FOV_Top_Right.y, MAP_WIDTH);
        p.FOV_Bottom_Right.x = p.postion.x + bottom_waist_length * cos(right_sight_bottom_angle * DEG2RAD);
        p.FOV_Bottom_Right.x = Frame_Area(p.FOV_Bottom_Right.x, MAP_LENGTH);
        p.FOV_Bottom_Right.y = p.postion.y - bottom_waist_length * sin(right_sight_bottom_angle * DEG2RAD);
        p.FOV_Bottom_Right.y = Frame_Area(p.FOV_Bottom_Right.y, MAP_WIDTH);

        p.FOV_Top_Left.x = p.postion.x + top_waist_length * cos(left_sight_top_angle * DEG2RAD);
        p.FOV_Top_Left.x = Frame_Area(p.FOV_Top_Left.x, MAP_LENGTH);
        p.FOV_Top_Left.y = p.postion.y - top_waist_length * sin(left_sight_top_angle * DEG2RAD);
        p.FOV_Top_Left.y = Frame_Area(p.FOV_Top_Left.y, MAP_WIDTH);
        p.FOV_Bottom_Left.x = p.postion.x + bottom_waist_length * cos(left_sight_bottom_angle * DEG2RAD);
        p.FOV_Bottom_Left.x = Frame_Area(p.FOV_Bottom_Left.x, MAP_LENGTH);
        p.FOV_Bottom_Left.y = p.postion.y - bottom_waist_length * sin(left_sight_bottom_angle * DEG2RAD); 
        p.FOV_Bottom_Left.y = Frame_Area(p.FOV_Bottom_Left.y, MAP_WIDTH); 
        
        p.FOV_corrdinate[0].x = p.FOV_Top_Left.x;
        p.FOV_corrdinate[0].y = p.FOV_Top_Left.y;
        p.FOV_corrdinate[1].x = p.FOV_Top_Right.x;
        p.FOV_corrdinate[1].y = p.FOV_Top_Right.y;
        p.FOV_corrdinate[2].x = p.FOV_Bottom_Right.x;
        p.FOV_corrdinate[2].y = p.FOV_Bottom_Right.y;
        p.FOV_corrdinate[3].x = p.FOV_Bottom_Left.x;
        p.FOV_corrdinate[3].y = p.FOV_Bottom_Left.y;
        
    }
}

bool Particle::CheckPointArea(ParticlePoint *tmp, int x, int y)
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

void Particle::FindFeaturePoint()
{
    ROS_INFO("FindFeaturePoint");
    for(auto& p : particles)
    {
        if(p.featurepoint_scan_line.size() != 0)
        {
            p.featurepoint_scan_line.clear();
        }
        
        int InnerMsg = 5;
        int OuterMsg = 800;
        int centerx = (p.FOV_Bottom_Right.x + p.FOV_Bottom_Left.x) / 2;
        int centery = (p.FOV_Bottom_Right.y + p.FOV_Bottom_Left.y) / 2;
        int Top_Left_dis = (int)(sqrt(pow((p.FOV_Top_Left.x - centerx),2) + pow((p.FOV_Top_Left.y - centery),2)));
        int Top_Right_dis = (int)(sqrt(pow((p.FOV_Top_Right.y - centerx),2) + pow((p.FOV_Top_Right.y - centery),2)));
        if(Top_Left_dis > Top_Right_dis)
        {
            OuterMsg = Top_Left_dis + 10;
        }
        else
        {
            OuterMsg = Top_Right_dis + 10;
        }
        int scan_line_cnt = 0;      //the number of scan_line
        for (float angle = p.FOV_dir +  - 90.0; angle <= p.FOV_dir + 91.0; angle = angle + 5.0)
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
                    p.featurepoint_scan_line.push_back(scan_tmp);
                    scan_line_cnt++;
                    break;
                }
                else
                {
                    if(CheckPointArea(&p, x, y))
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
                                    tmp.dis = sqrt(pow((x - p.postion.x),2) + pow((y - p.postion.y),2));
                                    tmp.x_dis = abs(x - p.postion.x);
                                    tmp.y_dis = abs(y - p.postion.y);
                                    scan_tmp.feature_point.push_back(tmp);
                                }
                            }
                            else
                            {
                                tmp.x = x;
                                tmp.y = y;
                                tmp.dis = sqrt(pow((x - p.postion.x),2) + pow((y - p.postion.y),2));
                                tmp.x_dis = abs(x - p.postion.x);
                                tmp.y_dis = abs(y - p.postion.y);
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
                        p.featurepoint_scan_line.push_back(scan_tmp);
                        scan_line_cnt++;
                        break;
                    }    
                }           
            }
        }
    }
}

void ParticleFilter::FindLandMarkInVirtualField(ParticlePoint particlepoint)
{
    // ROS_INFO("FindLandMarkInVirtualField");
    FieldLine_data FOV_function_data_tmp[4]; // To caculate FOV's linear function
    particlepoint.landmark_list.clear();
    int centerx = (particlepoint.FOV_Bottom_Right.x + particlepoint.FOV_Bottom_Left.x) / 2;
    int centery = (particlepoint.FOV_Bottom_Right.y + particlepoint.FOV_Bottom_Left.y) / 2;
    
    for(int j = 0; j < 4; j++)
    {
        Point FOV_tmp = particlepoint.FOV_corrdinate[j];
        Point FOV_tmp2 = particlepoint.FOV_corrdinate[0];
        Point FOV_tmp1 = Point(0,0);
        if(j<3)
        {
            FOV_tmp1 = particlepoint.FOV_corrdinate[j+1];
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
        particlepoint.landmark_list.push_back(landmark_tmp);
    }
}


double Particle::calcWeight(scan_line feature_point_observation_data, LineINF Line_observation_data)
{
    fwl = -1;
    factorweight = 1;
    float x_avg = 0.0;
    float y_avg = 0.0;
    if(use_feature_point)
    {
        for(auto& p : particles)
        {
            x_avg += p.postion.x;
            y_avg += p.postion.y;
            p.fitness_value = 0;
            p.weight = 0.0;
            int real_feature_point_cnt = 0;
            for(int j = 0; j < p.featurepoint_scan_line.size(); j++)//36
            {
                real_feature_point_cnt += feature_point_observation_data[j].feature_point.size();
                if(p.featurepoint_scan_line[j].feature_point.size() == feature_point_observation_data[j].feature_point.size())
                {
                    int scan_line_fitness = 0.0;
                    for(int k = 0; k < p.featurepoint_scan_line[j].feature_point.size(); k++)
                    {
                        if(feature_point_observation_data[j].feature_point[k].dis < 0)
                        {
                            if(p.featurepoint_scan_line[j].feature_point[k].dis < 0)
                            {
                                scan_line_fitness += MAP_MAX_LENGTH;
                            }
                        }
                        else
                        {
                            if(p.featurepoint_scan_line[j].feature_point[k].dis > 0)
                            {
                                int Dis_error = abs(feature_point_observation_data[j].feature_point[k].dis - p.featurepoint_scan_line[j].feature_point[k].dis);
                                Dis_error = abs(MAP_MAX_LENGTH - Dis_error);
                                scan_line_fitness = scan_line_fitness + Dis_error;
                            }
                        }
                    }
                    p.fitness_value += (scan_line_fitness / p.featurepoint_scan_line[j].feature_point.size());
                }
                else if(p.featurepoint_scan_line[j].feature_point.size() > feature_point_observation_data[j].feature_point.size())
                {
                    int scan_line_fitness = 0;
                    for(int k = 0; k < feature_point_observation_data[j].feature_point.size(); k++)
                    {
                        if(feature_point_observation_data[j].feature_point[k].dis < 0)
                        {
                            if(p.featurepoint_scan_line[j].feature_point[k].dis < 0)
                            {
                                scan_line_fitness += MAP_MAX_LENGTH;
                            }
                        }
                        else
                        {
                            if(p.featurepoint_scan_line[j].feature_point[k].dis > 0)
                            {
                                int Dis_error = abs(feature_point_observation_data[j].feature_point[k].dis - p.featurepoint_scan_line[j].feature_point[k].dis);
                                Dis_error = abs(MAP_MAX_LENGTH - Dis_error);
                                scan_line_fitness = scan_line_fitness + Dis_error;
                            }
                        }
                    }
                    p.fitness_value += (scan_line_fitness / p.featurepoint_scan_line[j].feature_point.size());
                }
                else
                {
                    int scan_line_fitness = 0;
                    for(int k = 0; k < p.featurepoint_scan_line[j].feature_point.size(); k++)
                    {
                        if(feature_point_observation_data[j].feature_point[k].dis < 0)
                        {
                            if(p.featurepoint_scan_line[j].feature_point[k].dis < 0)
                            {
                                scan_line_fitness += MAP_MAX_LENGTH;
                            }
                        }
                        else
                        {
                            if(p.featurepoint_scan_line[j].feature_point[k].dis > 0)
                            {
                                int Dis_error = abs(feature_point_observation_data[j].feature_point[k].dis - p.featurepoint_scan_line[j].feature_point[k].dis);
                                Dis_error = abs(MAP_MAX_LENGTH - Dis_error);
                                scan_line_fitness = scan_line_fitness + Dis_error;
                            }
                        }
                    }
                    p.fitness_value += (scan_line_fitness / feature_point_observation_data[j].feature_point.size());
                }
            } 
            p.weight = (float)p.fitness_value / ((float)(real_feature_point_cnt * MAP_MAX_LENGTH))+(float)p.likehood;
            factorweight *= exp(fwl)/(sqrt(2*M_PI))*SigmaIMU;
            p.wfactor = max(min(log(factorweight/p.weight),2),0.);
        }
    }
    if(use_lineinformation)
    {
        for(auto& p : particles)
        {
            x_avg += p.postion.x;
            y_avg += p.postion.y;
            vector<double> P;
            p.weight = 0.0;
            for(int m = 0; m < p.landmark_list.size(); m++)//計算虛擬地圖中的地標相似性
            {
                Pointdata robot = {Point(0,0),0};
                robot.pose = p.pos.pose;
                robot.angle = p.pos.angle;
                if((m -1) >= 0)
                {
                    double gx = robot.pose.x + p.landmark_list[m].distance * cos(robot.angle + p.landmark_list[m].Line_theta);
                    double gy = robot.pose.y + p.landmark_list[m].distance * sin(robot.angle + p.landmark_list[m].Line_theta);
                    p.landmark_list[m-1].mu << gx, gy;
                    //get the Jacobian with respect to the landmark position
                    Eigen::MatrixXd G;
                    Eigen::Vector2d g;
                    Measurement_model(p, m, g, G);
                    
                    //initialize the ekf for this landmark
                    MatrixXd Gi = G.inverse();
                    p.landmark_list[m-1].sigma = Gi * R * Gi.transpose();

                    Eigen::Vector2d expect_z;
                    Eigen::MatrixXd G;
                    Measurement_model(p,m, expect_z, G) ;
                    
                    //compute the measurement covariance
                    Eigen::MatrixXd sig = p.landmark_list[m - 1].sigma;
                    Eigen::MatrixXd Z   = G * sig * G.transpose() + R;
                    //calculate the Kalman gain K
                    Eigen::MatrixXd K = sig * G.transpose() * Z.inverse();
                    //calculat the error between the z and expected Z
                    Eigen::Vector2d z_actual;
                    if(Line_observation_data_flag == true)
                    {
                        if(m < Line_observation_data_Size-1)
                        {
                            ROS_INFO("Line_observation_data[m].distance = %f",Line_observation_data[m].distance);
                            ROS_INFO("Line_observation_data[m].Line_theta = %f",Line_observation_data[m].Line_theta);
                            z_actual << Line_observation_data[m].distance, Line_observation_data[m].Line_theta;
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
                    p.landmark_list[m-1].mu    = p.landmark_list[m-1].mu + K * z_diff;
                    p.landmark_list[m-1].sigma = p.landmark_list[m-1].sigma - K * G * sig;
                    ROS_INFO("particlepoint[%d].landmark_list[m-1].mu = %f",i,p.landmark_list[m-1].mu);    
                    ROS_INFO("particlepoint[%d].landmark_list[m-1].sigma = %f",i,p.landmark_list[m-1].sigma); 
                    //calculate the weight
                    double p = exp(-0.5*z_diff.transpose()*Z.inverse()*z_diff)/sqrt(2 * PI * Z.determinant());
                    // ROS_INFO("p = %f, exp(-0.5*z_diff.transpose()*Z.inverse()*z_diff) = %f,sqrt(2 * PI * Z.determinant()) = %f",p,exp(-0.5*z_diff.transpose()*Z.inverse()*z_diff),sqrt(2 * PI * Z.determinant()));    
                    P.push_back(p);
                    // p.likehood *= p;
                    // ROS_INFO("particlepoint[%d].likehood = %f",i,p.likehood);      
                }
            }
            double maxP = *max_element(P.begin(),P.end()); 
            p.weight = maxP;
            factorweight *= exp(fwl)/(sqrt(2*M_PI))*SigmaIMU;
            p.wfactor = max(min(log(factorweight/p.weight),2),0.);
        }
    }
    int totalweight = 0;
    totalweight += p.weight;
    if(use_feature_point && use_lineinformation)
    {
        totalweight = totalweight/2;
    }else{
        totalweight = totalweight;
    }
    particlepoint_compare = particles;
    sort(particlepoint_compare.begin(), particlepoint_compare.end(), greater<ParticlePoint>());
    FindRobotPosition(x_avg,y_avg);
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

void Particle::FindRobotPosition(float x_avg, float y_avg)
{
    ROS_INFO("FindRobotPosition");
    ROS_INFO("x_avg = %f",x_avg);
    ROS_INFO("y_avg = %f",y_avg);
    float x_varience = 0.0;
    float y_varience = 0.0;
    for(auto& p : particles)
    {
        float x_ = p.postion.x - x_avg;
        float y_ = p.postion.y - y_avg;
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
    for(auto& p : particles)
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

void Particle::CalcFOVArea_averagepos(int focus, int top, int bottom, int top_width, int bottom_width, float horizontal_head_angle)
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

void Particle::LandMarkMode(Landmarkmode mode)
{
    ROS_INFO("LandMarkMode");
    switch(mode)
    {
        case Landmarkmode::PARTICLEPIONT:
            for(auto& p : particles)
            {
                FindLandMarkInVirtualField(p);
            }
            break;
        case Landmarkmode::ROBOT:
            FindLandMarkInVirtualField(Robot_Position);
            break;
    }
}








