#include "main/Localizationmain.h"

Localization_main::Localization_main(ros::NodeHandle &nh)
{
    this->nh = &nh;
    image_transport::ImageTransport it(nh);

    nh_timer.setCallbackQueue(&queue_timer);
    queue_timer.callAvailable(ros::WallDuration());
    spinner_timer = new ros::AsyncSpinner(1, &queue_timer);
    spinner_timer->start();

    timer = nh_timer.createTimer(ros::Duration(0.96),&Localization_main::CalcStepFunction,this); 
    //timer_2 = nh_timer.createTimer(ros::Duration(1),&Localization_main::CalcLoopRateFunction,this); 
    //check_time.setTimerPass(250);
    timer.start();
    //timer_2.start();

    ImageLengthData_Subscriber = nh.subscribe("/vision/imagelength_data", 1, &Localization_main::GetImageLengthDataFunction,this);
    ObservationData_Subscriber = nh.subscribe("/vision/observation_data", 1, &Localization_main::GetObservationDataFunction,this);
    IMUData_Subscriber = nh.subscribe("/package/sensorpackage", 1, &Localization_main::GetIMUDataFunction,this);
    SendBodyAuto_Subscriber = nh.subscribe("/SendBodyAuto_Topic", 1, &Localization_main::GetSendBodyAutoFunction,this);
    ContinuousValue_Subscriber = nh.subscribe("/ChangeContinuousValue_Topic", 1, &Localization_main::GetContinuousValueFunction,this);
    WalkingParameter_Subscriber = nh.subscribe("/package/parameterdata", 1, &Localization_main::GetWalkingParameterFunction,this);
    SetRobotPos_Subscriber = nh.subscribe("/web/setrobot", 1, &Localization_main::GetSetRobotPosFunction,this);
    Start_Subscriber = nh.subscribe("/web/start", 10, &Localization_main::StartFunction,this);
    DIO_Ack_Subscriber = nh.subscribe("/package/FPGAack", 10, &Localization_main::DIOackFunction,this);

    GetVelocity_Subscriber = nh.subscribe("/GetVelocityValue_Topic", 10, &Localization_main::GetVelocityValue,this);
    GetIMUData_Subscriber = nh.subscribe("/imu/rpy/filtered", 10, &Localization_main::GetIMUData,this);
    RobotPos_Publisher = nh.advertise<tku_msgs::RobotPos>("/localization/robotpos", 1000);
    DrawRobotPos_Publisher = it.advertise("/localization/DrawRobotPos", 1);
    ParticlePoint_Publisher = it.advertise("/localization/ParticlePoint", 1);

    period_pre = 0.0;
    first_get_imu = false;
    is_start = false;
    change_pos_flag = false;
    loop_cnt = 0;

    loop_cnt_vector.clear();
}
Localization_main::~Localization_main()
{
    delete spinner_timer;
}
void Localization_main::GetIMUData(const geometry_msgs::Vector3Stamped &msg)
{
    imu_data.roll = msg.vector.x;
    imu_data.pitch = msg.vector.y;
    imu_data.yaw = msg.vector.z;
    // ROS_INFO("r = %f, p = %f, y = %f",imu_data.roll,imu_data.pitch,imu_data.yaw);
}
void Localization_main::GetVelocityValue(const tku_msgs::GetVelocity &msg)
{
    Velocity_value.straight = (float)msg.x;
    Velocity_value.drift = (float)msg.y;
    Velocity_value.rotational = (float)msg.thta;
    Velocity_value.moving = (float)msg.moving;
    Velocity_value.dt = (float)msg.dt;
}

void Localization_main::GetImageLengthDataFunction(const tku_msgs::ImageLengthData &msg)
{
    Camera_Focus = msg.focus;
    Image_Top_Length = msg.top;
    Image_Bottom_Length = msg.bottom;
    Image_Top_Width_Length = msg.top_width;
    Image_Bottom_Width_Length = msg.bottom_width;
    Horizontal_Head_Angle = msg.horizontal_head_angle;
    get_image_data = true;

}

void Localization_main::GetObservationDataFunction(const tku_msgs::ObservationData &msg)
{
    observation_data = msg;
    feature_point_observation_data.clear();
    Line_observation_data.clear();
    
    Point end_point;
    Point start_point;
    Point center_point;
    double Line_length;
    double Line_theta; 
    for(int i = 0; i < msg.landmark.size(); i++)
    {
        LineINF lineinf;
        lineinf.start_point = Point((int)msg.landmark[i].start_point.x,(int)msg.landmark[i].start_point.y);
        lineinf.end_point = Point((int)msg.landmark[i].end_point.x,(int)msg.landmark[i].end_point.y);
        // lineinf.center_point = Point((int)msg.landmark[i].center_point.x,(int)msg.landmark[i].center_point.y);
        // lineinf.Line_length = msg.landmark[i].Line_length;
        lineinf.Line_theta = msg.landmark[i].Line_theta;
        lineinf.distance = msg.landmark[i].relative_distance;
        lineinf.Nearest_point = Point((int)msg.landmark[i].Nearest_point.x,(int)msg.landmark[i].Nearest_point.y);
        Line_observation_data.push_back(lineinf);
    }
    Line_observation_data_Size = Line_observation_data.size();
    if(Line_observation_data.size() == 0)
    {
        Line_observation_data_flag = false;
    }else{
        Line_observation_data_flag = true;
    }  
    for(int i = 0; i < msg.scan_line.size(); i++)
    {
        scan_line scan_tmp;
        for(int j = 0; j < msg.scan_line[i].feature_point.size(); j++)
        {
            featuredata dis_tmp;
            dis_tmp.x_dis = (int)msg.scan_line[i].feature_point[j].x_dis;
            dis_tmp.y_dis = (int)msg.scan_line[i].feature_point[j].y_dis;
            dis_tmp.dis = (int)msg.scan_line[i].feature_point[j].dis;
            scan_tmp.feature_point.push_back(dis_tmp);
        }
        feature_point_observation_data.push_back(scan_tmp);
    }
}

void Localization_main::GetIMUDataFunction(const tku_msgs::SensorPackage &msg)
{
    if(!msg.IMUData.empty())
    {
        if(msg.IMUData[0] == 0.0 && msg.IMUData[1] == 0.0 && msg.IMUData[2] == 0.0)
        {
            Robot_Position.pos.angle = angle_pre[0];
        }
        else
        {
            if(!first_get_imu)
            {
                Robot_Position.pos.angle = normalize_angle(msg.IMUData[2]);
                angle_pre[0] = Robot_Position.pos.angle;
                angle_pre[1] = angle_pre[0] / 90.0;
                if(angle_pre[1] == 4)
                {
                    angle_pre[1] = 3;
                }
                first_get_imu = true;
            }
            else
            {
                float present_angle[2] = {0.0};
                present_angle[0] = normalize_angle(msg.IMUData[2]);
                present_angle[1] = present_angle[0] / 90.0;
                if(present_angle[1] == 4)
                {
                    present_angle[1] = 3;
                }

                double angle_error = present_angle[0] - angle_pre[0];
                if(abs(angle_error) < 90.0)
                {
                    Robot_Position.pos.angle = present_angle[0];
                    angle_pre[0] = Robot_Position.pos.angle;
                    angle_pre[1] = angle_pre[0] / 90.0;
                    if(angle_pre[1] == 4)
                    {
                        angle_pre[1] = 3;
                    }
                }
                else
                {
                    int quadrant_error = int(present_angle[1] - angle_pre[1]);
                    quadrant_error = abs(quadrant_error);
                    if(quadrant_error > 2)
                    {
                        if(angle_error < 0)
                        {
                            angle_error = 360.0 + angle_error;
                        }
                        else
                        {
                            angle_error = 360 - angle_error;
                        }

                        if(angle_error < 90.0)
                        {
                            Robot_Position.pos.angle = present_angle[0];
                            angle_pre[0] = Robot_Position.pos.angle;
                            angle_pre[1] = angle_pre[0] / 90.0;
                            if(angle_pre[1] == 4)
                            {
                                angle_pre[1] = 3;
                            }
                        }
                        else
                        {
                            Robot_Position.pos.angle = angle_pre[0];
                        }
                    }
                    else
                    {
                         Robot_Position.pos.angle = angle_pre[0];
                    }
                }
            }
        }
        //ROS_INFO("Angle_1 = %f",Robot_Position.pos.angle);
    }
}

void Localization_main::GetSendBodyAutoFunction(const tku_msgs::Interface &msg)
{
    if(msg.walking_mode == int(WalkingMode::ContinuousStep) || msg.walking_mode == int(WalkingMode::ContinuousStep_second) || msg.walking_mode == int(WalkingMode::ContinuousStep_third))
    {
        continuous_x = msg.x;
        continuous_y = msg.y;
        continuous_theta = msg.theta;
        ROS_INFO("continuous_x_start = %d",continuous_x);
    }
    else
    {
        sendbodyauto_x = msg.x;
        sendbodyauto_y = msg.y;
        sendbodyauto_theta = msg.theta;
    }
}

void Localization_main::GetContinuousValueFunction(const tku_msgs::Interface &msg)
{
    continuous_x = msg.x;
    continuous_y = msg.y;
    continuous_theta = msg.theta;
    ROS_INFO("continuous_x_forever = %d",continuous_x);
}

void Localization_main::GetWalkingParameterFunction(const tku_msgs::Parameter_message &msg)
{
    float period = float(msg.Period_T) / 1000.0;
    if(period_pre != period)
    {
        timer.setPeriod(ros::Duration(period));
        period_pre = period;
    }
    ROS_INFO("period = %f",period);
} 

void Localization_main::GetSetRobotPosFunction(const tku_msgs::SetRobotPos &msg) 
{
    if(msg.number == 0)
    {
        Robot_Position.pos.pose = Point(msg.x,msg.y);
        Robot_Position.pos.angle = msg.dir;
        robot_pos_x_init = msg.x;
        robot_pos_y_init = msg.y;
        robot_pos_dir_init = msg.dir;

        init_robot_pos_x = msg.x;
        init_robot_pos_y = msg.y;
        init_robot_pos_dir = msg.dir;
    }
}

void Localization_main::StartFunction(const std_msgs::Bool &msg)
{
    if(msg.data)
    {
        ROS_INFO("start");
        is_start = true;
    }
    else
    {
        ROS_INFO("stop");
        is_start = false;
    }
}

void Localization_main::DIOackFunction(const std_msgs::Int16 &msg)
{
    // if(!isInitialize())return;
    if(msg.data & 0x10)
    {
        is_start = true;
        change_pos_flag = true;
    }
    else
    {
        is_start = false;
    }
}

void Localization_main::CalcStepFunction(const ros::TimerEvent& event) 
{
    //ROS_INFO("INterRupt!!!!!!!!!");
    if(!Step_flag)
    {  
        Step_flag = true;
        //ROS_INFO("Step_flag = true!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "localization");

    ros::NodeHandle nh;
	Localization_main *localization_main;
    localization_main = nullptr;
    //set the robot position
    robot_pos_x_init = 750; 
    robot_pos_y_init = 213;
    robot_pos_dir_init = 0.0;

    bool reset_flag = false;

    ros::spinOnce();

    ros::Rate loop_rate(30);

    while (nh.ok())
    {
        if(localization_main == nullptr)
        {
            localization_main = new Localization_main(nh);
        }
        else
        {
            if(localization_main->is_start)
            {
                reset_flag = true;
                if(localization_main->get_image_data)
                {
                    switch(m_state)
                    {
                        case P_INIT:
                            //ROS_INFO("P_INIT");
                            localization_main->strategy_init();
                            m_state = P_LOCALIZATION;
                            break;
                        case P_LOCALIZATION:
                            //ROS_INFO("P_LOCALIZATION");
                            localization_main->strategy_main();
                            break;
                    }
                }
                localization_main->loop_cnt++;
            }
            else
            {
                if(reset_flag)
                {
                    m_state = P_INIT;
                    delete localization_main;
                    localization_main = nullptr;
                    reset_flag = false;
                }
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

// vector<int> Localization_main::GetAction()
// {
    
// }

void Localization_main::strategy_init()
{
    if(Robot_Position.pos.pose.x < 0 && Robot_Position.pos.pose.y < 0)
    {
        Robot_Position.pos.pose.x = robot_pos_x_init;
        Robot_Position.pos.pose.y = robot_pos_y_init;
        Robot_Position.pos.angle = robot_pos_dir_init;

    }
     
    Soccer_Field = DrawField();
    ParticlePointInitialize(field_list.size());
    ROS_INFO("robot_pos_x_init = %d robot_pos_y_init = %d robot_pos_dir_init = %f",robot_pos_x_init,robot_pos_y_init,robot_pos_dir_init);
    observation_data.imagestate = false;
}

void Localization_main::strategy_main()
{
    // imshow("Soccer_Field",Soccer_Field);
    // ROS_INFO("straight = %f ,drift = %f ,rotational = %f,dt = %f",Velocity_value.straight,Velocity_value.drift,Velocity_value.rotational,Velocity_value.dt); 
    if(!observation_data.imagestate || !Line_observation_data_flag)
    {
        NoLookField(Velocity_value);
        CalcFOVArea_averagepos(Camera_Focus, Image_Top_Length, Image_Bottom_Length, Image_Top_Width_Length, Image_Bottom_Width_Length, Horizontal_Head_Angle);
        //imshow("FOV_Field",DrawParticlePoint());
        // imshow("RobotPos",DrawRobotPos());
    }
    else
    {
        StatePredict(Velocity_value,first_loop_flag);
        if(first_loop_flag)
        {
            first_loop_flag = false;
        }
        CalcFOVArea(Camera_Focus, Image_Top_Length, Image_Bottom_Length, Image_Top_Width_Length, Image_Bottom_Width_Length, Horizontal_Head_Angle);
        FindFeaturePoint();
        LandMarkMode(Landmarkmode::PARTICLEPIONT);
        if(observation_data.scan_line.size() > 0 && observation_data.landmark.size()>=0)
        {
            FindBestParticle(&feature_point_observation_data[0], &Line_observation_data[0]);
            CalcFOVArea_averagepos(Camera_Focus, Image_Top_Length, Image_Bottom_Length, Image_Top_Width_Length, Image_Bottom_Width_Length, Horizontal_Head_Angle);
            LandMarkMode(Landmarkmode::ROBOT);
        }
        if(first_loop_flag)
        {
            first_loop_flag = false;
        }
        else
        {
            KLD_Sampling();
            resample();
        }
        
        //observation_data.clear();
        // imshow("FOV_Field",DrawParticlePoint());
        // namedWindow("ParticlePoint",WINDOW_NORMAL);
        // imshow("ParticlePoint",DrawParticlePoint());
        // namedWindow("RobotPos",WINDOW_AUTOSIZE);
        // imshow("RobotPos",DrawRobotPos());
    }
    //FOV_Field = DrawFOV();
    //particlepoint.clear();

    robot_pos.x = Robot_Position.pos.pose.x;
    robot_pos.y = Robot_Position.pos.pose.y;
    robot_pos.dir = Robot_Position.pos.angle;
    RobotPos_Publisher.publish(robot_pos);

    // tool->Delay(4000);
    //ROS_INFO("distance = %d",Robot_Position.featurepoint[18].dis);
    //ROS_INFO("distance_y = %d",Robot_Position.featurepoint[18].y_dis);
    ROS_INFO("Robot_pos_x = %d",Robot_Position.pos.pose.x);
    ROS_INFO("Robot_pos_y = %d",Robot_Position.pos.pose.y);
    ROS_INFO("Robot_pos_dir = %f",Robot_Position.pos.angle);
    // tool->Delay(5000);
    /*ROS_INFO("FOV_Top_Right = %d",Robot_Position.FOV_Top_Right.x);
    ROS_INFO("FOV_Top_Right = %d",Robot_Position.FOV_Top_Right.y);
    ROS_INFO("FOV_Bottom_Right = %d",Robot_Position.FOV_Bottom_Right.x);
    ROS_INFO("FOV_Bottom_Right = %d",Robot_Position.FOV_Bottom_Right.y);
    ROS_INFO("FOV_Top_Left = %d",Robot_Position.FOV_Top_Left.x);
    ROS_INFO("FOV_Top_Left = %d",Robot_Position.FOV_Top_Left.y);
    ROS_INFO("FOV_Bottom_Left = %d",Robot_Position.FOV_Bottom_Left.x);
    ROS_INFO("FOV_Bottom_Left = %d",Robot_Position.FOV_Bottom_Left.y);*/

    
    msg_DrawRobotPos = cv_bridge::CvImage(std_msgs::Header(), "bgr8", DrawRobotPos()).toImageMsg();
    msg_ParticlePoint = cv_bridge::CvImage(std_msgs::Header(), "bgr8", DrawParticlePoint()).toImageMsg();

    DrawRobotPos_Publisher.publish(msg_DrawRobotPos);
    ParticlePoint_Publisher.publish(msg_ParticlePoint);
    waitKey(10);
}

// void Localization_main::strategy_main()
// {
//     //imshow("Soccer_Field",Soccer_Field);
//     if(!observation_data.imagestate)
//     {
//         NoLookField(Velocityvalue);
//         CalcFOVArea_averagepos(Camera_Focus, Image_Top_Length, Image_Bottom_Length, Image_Top_Width_Length, Image_Bottom_Width_Length, Horizontal_Head_Angle);
//         //imshow("FOV_Field",DrawParticlePoint());
//         // imshow("RobotPos",DrawRobotPos());
//     }
//     else
//     {
//         step_count = 0;
//         if(first_loop_flag)
//         {
//             first_loop_flag = false;
//         }
//         else
//         {
//             KLD_Sampling();
//             StatePredict(Velocityvalue);
//         }
//         CalcFOVArea(Camera_Focus, Image_Top_Length, Image_Bottom_Length, Image_Top_Width_Length, Image_Bottom_Width_Length, Horizontal_Head_Angle);
//         FindFeaturePoint();
//         LandMarkMode(Landmarkmode::PARTICLEPIONT);
//         if(observation_data.scan_line.size() > 0)
//         {
//             FindBestParticle(&feature_point_observation_data[0], &Line_observation_data[0]);
//             CalcFOVArea_averagepos(Camera_Focus, Image_Top_Length, Image_Bottom_Length, Image_Top_Width_Length, Image_Bottom_Width_Length, Horizontal_Head_Angle);
//             LandMarkMode(Landmarkmode::ROBOT);
//         }
//         //observation_data.clear();
//         //imshow("FOV_Field",DrawParticlePoint());
//         // namedWindow("ParticlePoint",WINDOW_NORMAL);
//         // imshow("ParticlePoint",DrawParticlePoint());
//         // namedWindow("RobotPos",WINDOW_AUTOSIZE);
//         // imshow("RobotPos",DrawRobotPos());
//     }
//     //FOV_Field = DrawFOV();
//     //particlepoint.clear();

//     robot_pos.x = Robot_Position.pos.pose.x;
//     robot_pos.y = Robot_Position.pos.pose.y;
//     robot_pos.dir = Robot_Position.pos.angle;
//     RobotPos_Publisher.publish(robot_pos);

//     //tool->Delay(4000);
//     //ROS_INFO("distance = %d",Robot_Position.featurepoint[18].dis);
//     //ROS_INFO("distance_y = %d",Robot_Position.featurepoint[18].y_dis);
//     //ROS_INFO("Robot_pos_x = %d",Robot_Position.pos.pose.x);
//     //ROS_INFO("Robot_pos_y = %d",Robot_Position.pos.pose.y);

//     /*ROS_INFO("FOV_Top_Right = %d",Robot_Position.FOV_Top_Right.x);
//     ROS_INFO("FOV_Top_Right = %d",Robot_Position.FOV_Top_Right.y);
//     ROS_INFO("FOV_Bottom_Right = %d",Robot_Position.FOV_Bottom_Right.x);
//     ROS_INFO("FOV_Bottom_Right = %d",Robot_Position.FOV_Bottom_Right.y);
//     ROS_INFO("FOV_Top_Left = %d",Robot_Position.FOV_Top_Left.x);
//     ROS_INFO("FOV_Top_Left = %d",Robot_Position.FOV_Top_Left.y);
//     ROS_INFO("FOV_Bottom_Left = %d",Robot_Position.FOV_Bottom_Left.x);
//     ROS_INFO("FOV_Bottom_Left = %d",Robot_Position.FOV_Bottom_Left.y);*/

    
//     msg_DrawRobotPos = cv_bridge::CvImage(std_msgs::Header(), "bgr8", DrawRobotPos()).toImageMsg();
//     msg_ParticlePoint = cv_bridge::CvImage(std_msgs::Header(), "bgr8", DrawParticlePoint()).toImageMsg();

//     DrawRobotPos_Publisher.publish(msg_DrawRobotPos);
//     ParticlePoint_Publisher.publish(msg_ParticlePoint);
//     waitKey(10);
// }

