#include "main/Localizationmain.h"

Localization_main::Localization_main(ros::NodeHandle &nh)
{
    this->nh = &nh;

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

    LocalizationPos_Publisher = nh.advertise<tku_msgs::LocalizationPos>("/localization/localizationpos", 1000);

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
            Robot_Position.angle = angle_pre[0];
        }
        else
        {
            if(!first_get_imu)
            {
                Robot_Position.angle = Angle_Adjustment(msg.IMUData[2]);
                angle_pre[0] = Robot_Position.angle;
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
                present_angle[0] = Angle_Adjustment(msg.IMUData[2]);
                present_angle[1] = present_angle[0] / 90.0;
                if(present_angle[1] == 4)
                {
                    present_angle[1] = 3;
                }

                double angle_error = present_angle[0] - angle_pre[0];
                if(abs(angle_error) < 90.0)
                {
                    Robot_Position.angle = present_angle[0];
                    angle_pre[0] = Robot_Position.angle;
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
                            Robot_Position.angle = present_angle[0];
                            angle_pre[0] = Robot_Position.angle;
                            angle_pre[1] = angle_pre[0] / 90.0;
                            if(angle_pre[1] == 4)
                            {
                                angle_pre[1] = 3;
                            }
                        }
                        else
                        {
                            Robot_Position.angle = angle_pre[0];
                        }
                    }
                    else
                    {
                         Robot_Position.angle = angle_pre[0];
                    }
                }
            }
        }
        //ROS_INFO("Angle_1 = %f",Robot_Position.angle);
    }
}

void Localization_main::GetSendBodyAutoFunction(const tku_msgs::Interface &msg)
{
    if(msg.walking_mode == int(WalkingMode::ContinuousStep) || msg.walking_mode == int(WalkingMode::ContinuousStep_second) || msg.walking_mode == int(WalkingMode::ContinuousStep_third))
    {
        continuous_x = msg.x;
        continuous_y = msg.y;
        // ROS_INFO("continuous_x_start = %d",continuous_x);
    }
    else
    {
        sendbodyauto_x = msg.x;
        sendbodyauto_y = msg.y;
    }
}

void Localization_main::GetContinuousValueFunction(const tku_msgs::Interface &msg)
{
    continuous_x = msg.x;
    continuous_y = msg.y;
    // ROS_INFO("continuous_x_forever = %d",continuous_x);
}

void Localization_main::GetWalkingParameterFunction(const tku_msgs::Parameter_message &msg)
{
    float period = float(msg.Period_T) / 1000.0;
    if(period_pre != period)
    {
        timer.setPeriod(ros::Duration(period));
        period_pre = period;
    }
    // ROS_INFO("period = %f",period);
} 

void Localization_main::GetSetRobotPosFunction(const tku_msgs::SetRobotPos &msg) 
{
    // if(msg.number == 0) // must change
    // {
    Robot_Position.postion.X = msg.x;
    Robot_Position.postion.Y = msg.y;
    Robot_Position.angle = msg.dir;
    robot_pos_x_init = msg.x;
    robot_pos_y_init = msg.y;
    robot_pos_dir_init = msg.dir;
    // }
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

void Localization_main::strategy_init()
{
    if(Robot_Position.postion.X < 0 && Robot_Position.postion.Y < 0)
    {
        Robot_Position.postion.X = robot_pos_x_init;
        Robot_Position.postion.Y = robot_pos_y_init;
        Robot_Position.angle = robot_pos_dir_init;
    }
    ParticlePointinit();
    Soccer_Filed = DrawFiled();

    observation_data.imagestate = false;
}

void Localization_main::strategy_main()
{
    //imshow("Soccer_Filed",Soccer_Filed);
    if(!observation_data.imagestate)
    {
        NoLookFiled();
        CalcFOVArea_averagepos(Camera_Focus, Image_Top_Length, Image_Bottom_Length, Image_Top_Width_Length, Image_Bottom_Width_Length, Horizontal_Head_Angle);
        // //imshow("FOV_Filed",DrawParticlePoint());
        // imshow("RobotPos",DrawRobotPos());
    }
    else
    {
        step_count = 0;
        if(first_loop_flag)
        {
            first_loop_flag = false;
        }
        else
        {
            KLD_Sampling();
            StatePredict();
        }
        CalcFOVArea(Camera_Focus, Image_Top_Length, Image_Bottom_Length, Image_Top_Width_Length, Image_Bottom_Width_Length, Horizontal_Head_Angle);
        FindFeaturePoint();
        if(observation_data.scan_line.size() > 0)
        {
            FindBestParticle(&feature_point_observation_data[0]);
            CalcFOVArea_averagepos(Camera_Focus, Image_Top_Length, Image_Bottom_Length, Image_Top_Width_Length, Image_Bottom_Width_Length, Horizontal_Head_Angle);
        }
        //observation_data.clear();
        //imshow("FOV_Filed",DrawParticlePoint());
        // namedWindow("ParticlePoint",WINDOW_NORMAL);
        // imshow("ParticlePoint",DrawParticlePoint());
        // namedWindow("RobotPos",WINDOW_AUTOSIZE);
        // imshow("RobotPos",DrawRobotPos());
    }
    //FOV_Filed = DrawFOV();
    //particlepoint.clear();

    localization_pos.x = Robot_Position.postion.X;
    localization_pos.y = Robot_Position.postion.Y;
    localization_pos.dir = Robot_Position.angle;
    localization_pos.weight = Robot_Position.weight;
    LocalizationPos_Publisher.publish(localization_pos);

    //tool->Delay(4000);
    //ROS_INFO("distance = %d",Robot_Position.featurepoint[18].dis);
    //ROS_INFO("distance_y = %d",Robot_Position.featurepoint[18].y_dis);
    //ROS_INFO("Robot_pos_x = %d",Robot_Position.postion.X);
    //ROS_INFO("Robot_pos_y = %d",Robot_Position.postion.Y);

    /*ROS_INFO("FOV_Top_Right = %d",Robot_Position.FOV_Top_Right.X);
    ROS_INFO("FOV_Top_Right = %d",Robot_Position.FOV_Top_Right.Y);
    ROS_INFO("FOV_Bottom_Right = %d",Robot_Position.FOV_Bottom_Right.X);
    ROS_INFO("FOV_Bottom_Right = %d",Robot_Position.FOV_Bottom_Right.Y);
    ROS_INFO("FOV_Top_Left = %d",Robot_Position.FOV_Top_Left.X);
    ROS_INFO("FOV_Top_Left = %d",Robot_Position.FOV_Top_Left.Y);
    ROS_INFO("FOV_Bottom_Left = %d",Robot_Position.FOV_Bottom_Left.X);
    ROS_INFO("FOV_Bottom_Left = %d",Robot_Position.FOV_Bottom_Left.Y);*/

    waitKey(1);
}

