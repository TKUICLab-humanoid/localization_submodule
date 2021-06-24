#include <ros/ros.h>
#include <math.h>
#include <fstream>  

//----------------libs-------------------
#include "tku_libs/strategy_info.h"
#include "tku_libs/TKU_tool.h"
#include "tku_libs/RosCommunication.h"
#include "tku_libs/WalkContinuouse.h"
#include "tku_libs/RobotCupInfo.h"
#include "tku_libs/Ros2MultiCommunication.h"
#include "tku_libs/robocup_referee/referee_client.h"

#include "tku_msgs/LocalizationPos.h"
#include "tku_msgs/RobotPos.h"
#include "tku_msgs/SoccerPos.h"

#define PI               M_PI
#define DEG2RAD          M_PI/180
#define highThreshold    0.8
#define lowThreshold     0.5


using namespace robocup_referee;

struct Coordinate
{
    int x;
    int y;
};

struct RobotPositionData
{
	float theta;
	float weight;

	Coordinate position;
};

struct VisionData
{
	bool existFlag;
	int x;
	int y;
	int width;
	int height;
	int size;
	int x_dis;
	int y_dis;
	int dis;
	float theta;
};

struct GoalData
{
	bool existFlag;
	int x[2];
	int y[2];
	int width[2];
	int height[2];
	int size[2];
	int cnt;
};

class MultiRobotLocalization
{
	public:
		MultiRobotLocalization(ros::NodeHandle &nh);
		~MultiRobotLocalization();

		void strategyMain();
		void calculateSelfPosition();
		void calculateObjectPosition();
		void calculateSoccerPosition();
		void calculatePartnerPosition();
		void getObjectInfo();
		float Angle_Adjustment(float angle);
		void transmitRoboCupInfo();
		void saveData();
		void publishPosition();
		void robotPositionDataInitialize();
		void soccerDataInitialize();
		void partnerDataInitialize();
		void enemyDataInitialize();
		void goalDataInitialize();
		void saveDataInitialize();

	public:
		void getLocalizationPositionFunction(const tku_msgs::LocalizationPos &msg);

	private:
		StrategyInfoInstance *strategy_info;
		ToolInstance *tool;
		RosCommunicationInstance *ros_com;
		WalkContinuouseInstance *walk_con;
		Ros2MultiCommunication *ros2MultiCom;
		RobotCupInfo *robotCupInfo;
		RefereeClient client;

		ros::Publisher RobotPos_Publisher;
		ros::Publisher SoccerPos_Publisher;

		ros::Subscriber localizationPos_Subscriber;

		tku_msgs::RobotPos selfPosition;

	private:
		int loopTimes;
		std::string filePath;

		RobotPositionData localizationPosition;
		VisionData soccer;
		VisionData partner;
		VisionData enemy;
		GoalData goal;
		
		std::map<std::string, CharacterInfo*>::iterator itMyself;
};
