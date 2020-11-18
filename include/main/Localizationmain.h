#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include "tku_msgs/ImageLengthData.h"
#include "tku_msgs/ObservationData.h"
#include "tku_msgs/SensorPackage.h"
#include "tku_msgs/Interface.h"
#include "tku_msgs/Parameter_message.h"
#include "tku_msgs/RobotPos.h"
#include "tku_msgs/SetRobotPos.h"

//#include "tku_libs/TKU_tool.h"
#include "tku_libs/RosCommunication.h"

#include "Drawing/Drawing.h"


#define PI 3.14159265
#define DEG2RAD  M_PI/180

using namespace std;
using namespace cv;

int robot_pos_x_init;
int robot_pos_y_init;
float robot_pos_dir_init;

enum state
{
	P_INIT, 
	P_LOCALIZATION
};
state m_state = P_INIT;
class Localization_main : public Drawing
{
    public:
        Localization_main(ros::NodeHandle &nh);
        ~Localization_main();
        void strategy_main();
        void strategy_init();

        void SaveLoopRate();
        void SaveRobotPosERR();
		string DtoS(double value);
    public:
        void GetImageLengthDataFunction(const tku_msgs::ImageLengthData &msg);
        void GetObservationDataFunction(const tku_msgs::ObservationData &msg);
        void GetIMUDataFunction(const tku_msgs::SensorPackage &msg);
        void GetSendBodyAutoFunction(const tku_msgs::Interface &msg);
        void GetContinuousValueFunction(const tku_msgs::Interface &msg);
        void GetWalkingParameterFunction(const tku_msgs::Parameter_message &msg);
        void GetSetRobotPosFunction(const tku_msgs::SetRobotPos &msg);
        void StartFunction(const std_msgs::Bool& msg);
		void DIOackFunction(const std_msgs::Int16 &msg);

        void CalcStepFunction(const ros::TimerEvent& event);
        void CalcLoopRateFunction(const ros::TimerEvent& event);
    public:
        bool get_image_data = false;
        bool is_start;
        int loop_cnt;
        vector<int> loop_cnt_vector;
        vector<Distance> robot_pos_error_data;
        vector<coordinate> robot_pos_data;
        vector<float> weight_vector;
    private:
        ros::NodeHandle *nh;
        ros::NodeHandle nh_timer;
        ros::CallbackQueue queue_timer;
        ros::AsyncSpinner *spinner_timer;

        ros::Timer timer;
        ros::Timer timer_2;

        tku_msgs::SensorSet IMUDataRequest;
        tku_msgs::RobotPos robot_pos;
        tku_msgs::ObservationData observation_data;

        ros::Subscriber ImageLengthData_Subscriber;
        ros::Subscriber ObservationData_Subscriber;
        ros::Subscriber IMUData_Subscriber;
        ros::Subscriber SendBodyAuto_Subscriber;
        ros::Subscriber ContinuousValue_Subscriber;
        ros::Subscriber WalkingParameter_Subscriber;
        ros::Subscriber SetRobotPos_Subscriber;
        ros::Subscriber Start_Subscriber;
		ros::Subscriber DIO_Ack_Subscriber;

        ros::Publisher  IMUDataRequest_Publisher; 
        ros::Publisher  RobotPos_Publisher;
    private:
        int Camera_Focus;
        int Image_Top_Length;
        int Image_Bottom_Length;
        int Image_Top_Width_Length;
        int Image_Bottom_Width_Length;
        int Head_position;
        
        float Horizontal_Head_Angle;

        float period_pre;
        float angle_pre[2] = {0.0};  //[0] is value,[1] is quadrant

        bool first_loop_flag = true;
        bool first_get_imu;

        vector<scan_line> feature_point_observation_data;

        Mat FOV_Filed;
};

