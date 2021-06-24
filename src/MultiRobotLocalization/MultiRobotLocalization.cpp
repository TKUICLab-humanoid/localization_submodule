#include "MultiRobotLocalization/MultiRobotLocalization.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "multirobotlocalization");
	ros::NodeHandle nh;
	MultiRobotLocalization MultiRobotLocalization(nh);

	ros::Rate loop_rate(30);

	while (nh.ok())
	{
		// if(system("clear"))
		// {
		// 	std::cerr << "Failed to clear terminal" << std::endl;
		// 	exit(EXIT_FAILURE);
		// }
		ros::spinOnce();
		MultiRobotLocalization.strategyMain();
		loop_rate.sleep();
	}
	return 0;
}

void MultiRobotLocalization::strategyMain()
{
	std::cout << &client.getGameState() << std::endl;
  	if(strategy_info->getStrategyStart() || client.getGameState().getActualGameState() != -1) //strategy start running
	{
		switch(client.getGameState().getActualGameState())
		{
			case 0:
				ROS_INFO("Initial");
				break;
			case 1:
				ROS_INFO("Ready");
				break;
			case 2:
				ROS_INFO("Set");
				break; 
			case 3:
				ROS_INFO("Playing");
				break;
			case 4:
				ROS_INFO("Finished");
				break;
			default:
				ROS_INFO("None");
				break;
		}
		calculateSelfPosition();
		calculateObjectPosition();
		transmitRoboCupInfo();
		publishPosition();
  	}
	else //strategy not running
	{
		ROS_INFO("Unconnect GameController");
	}
}

void MultiRobotLocalization::calculateSelfPosition()
{
	int weightCnt;
	float robotX;
	float robotY;
	float robotTheta;
	float weight;

	if(localizationPosition.weight >= highThreshold)
	{
		ROS_INFO("RobotPosition Section 1; Weight above highThreshold");
		robotX = localizationPosition.position.x;
		robotY = localizationPosition.position.y;
		robotTheta = localizationPosition.theta;
		weight = localizationPosition.weight;
	}
	else if(localizationPosition.weight < highThreshold && localizationPosition.weight >= lowThreshold)
	{
		ROS_INFO("RobotPosition Section 2; Weight under highThreshold and above lowThreshold");
		robotX = localizationPosition.position.x * localizationPosition.weight;
		robotY = localizationPosition.position.y * localizationPosition.weight;
		robotTheta = localizationPosition.theta;
		weight = localizationPosition.weight;
		weightCnt = 1;

		for(std::map<std::string, CharacterInfo*>::iterator it = robotCupInfo->characterInfo->who.begin(); it != robotCupInfo->characterInfo->who.end(); it++)
		{
			if(it->first != itMyself->first && it->first != itMyself->second->which_robot)
			{
				if(it->second->partner[itMyself->second->which_robot].exist_flag == true && it->second->weight >= localizationPosition.weight)
				{
					robotX += (it->second->partner[itMyself->second->which_robot].global.x_pos * it->second->weight);
					robotY += (it->second->partner[itMyself->second->which_robot].global.y_pos * it->second->weight);
					weight += it->second->weight;
					weightCnt++;
				}
			}
		}

		if(weightCnt >= 2)
		{
			robotX /= weight;
			robotY /= weight;
			weight /= weightCnt;
		}
		else
		{
			robotX = localizationPosition.position.x;
			robotY = localizationPosition.position.y;
			robotTheta = localizationPosition.theta;
			weight = localizationPosition.weight;
		}
	}
	else if(localizationPosition.weight < lowThreshold)
	{
		ROS_INFO("RobotPosition Section 3; Weight under lowThreshold");
		robotX = 0.0;
		robotY = 0.0;
		robotTheta = localizationPosition.theta;
		weight = 0.0;
		weightCnt = 0;

		for(std::map<std::string, CharacterInfo*>::iterator it = robotCupInfo->characterInfo->who.begin(); it != robotCupInfo->characterInfo->who.end(); it++)
		{
			if(it->first != itMyself->first && it->first != itMyself->second->which_robot)
			{
				if(it->second->partner[itMyself->second->which_robot].exist_flag == true && it->second->weight >= lowThreshold)
				{
					robotX += (it->second->partner[itMyself->second->which_robot].global.x_pos * it->second->weight);
					robotY += (it->second->partner[itMyself->second->which_robot].global.y_pos * it->second->weight);
					weight += it->second->weight;
					weightCnt++;
				}
			}
		}

		if(weightCnt >= 1)
		{
			robotX /= weight;
			robotY /= weight;
			weight /= weightCnt;
		}
		else
		{
			robotX = localizationPosition.position.x;
			robotY = localizationPosition.position.y;
			robotTheta = localizationPosition.theta;
			weight = localizationPosition.weight;
		}
	}
	
	itMyself->second->weight = weight;
	itMyself->second->global.x_pos = (int)robotX;
	itMyself->second->global.y_pos = (int)robotY;
	itMyself->second->global.theta = robotTheta;
	itMyself->second->local.x_pos = (int)robotX;
	itMyself->second->local.y_pos = (int)robotY;
	itMyself->second->local.theta = robotTheta;

	selfPosition.x = (int)robotX;
	selfPosition.y = (int)robotY;
	selfPosition.dir = robotTheta;

	ROS_INFO("selfPosition.x: %d", selfPosition.x);
	ROS_INFO("selfPosition.y: %d", selfPosition.y);
	ROS_INFO("selfPosition.dir: %f\n", selfPosition.dir);
}

void MultiRobotLocalization::calculateObjectPosition()
{
	getObjectInfo();

	calculateSoccerPosition();

	calculatePartnerPosition();
}

void MultiRobotLocalization::calculateSoccerPosition()
{
	bool soccerExistFlag;
	int soccerCnt;
	int weightCnt;
	int distanceX;
	int distanceY;
	float soccerTmpX;
	float soccerTmpY;
	float soccerTmpTheta;
	float soccerGlobalX;
	float soccerGlobalY;
	float soccerGlobalTheta;
	float soccerLocalX;
	float soccerLocalY;
	float soccerLocalTheta;
	float soccerWeight;
	float weight;
	
	if(soccer.existFlag)
	{
		soccerTmpTheta = Angle_Adjustment(selfPosition.dir + soccer.theta);
		soccerTmpX = selfPosition.x + soccer.dis * cos(soccerTmpTheta * DEG2RAD);
		soccerTmpY = selfPosition.y - soccer.dis * sin(soccerTmpTheta * DEG2RAD);

		if(localizationPosition.weight >= highThreshold)
		{
			ROS_INFO("SoccerPosition Section 1; Weight above highThreshold");
			soccerGlobalX = soccerTmpX;
			soccerGlobalY = soccerTmpY;
			soccerGlobalTheta = soccerTmpTheta;
			soccerLocalX = soccer.x_dis;
			soccerLocalY = soccer.y_dis;
			soccerLocalTheta = soccer.theta;
		}
		else if(localizationPosition.weight < highThreshold && localizationPosition.weight >= lowThreshold)
		{
			ROS_INFO("SoccerPosition Section 2; Weight under highThreshold and above lowThreshold");
			soccerGlobalX = soccerTmpX * localizationPosition.weight;
			soccerGlobalY = soccerTmpY * localizationPosition.weight;
			weight = localizationPosition.weight;
			weightCnt = 1;

			for(std::map<std::string, CharacterInfo*>::iterator it = robotCupInfo->characterInfo->who.begin(); it != robotCupInfo->characterInfo->who.end(); it++)
			{
				if(it->first != itMyself->first && it->first != itMyself->second->which_robot)
				{
					if(it->second->object["soccer"].exist_flag == true && it->second->weight >= localizationPosition.weight)
					{
						soccerGlobalX += (it->second->object["soccer"].global.x_pos * it->second->weight);
						soccerGlobalY += (it->second->object["soccer"].global.y_pos * it->second->weight);
						weight += it->second->weight;
						weightCnt++;
					}
				}
			}

			if(weightCnt >= 2)
			{
				soccerGlobalX /= weight;
				soccerGlobalY /= weight;
				distanceX = soccerGlobalX - selfPosition.x;
				distanceY = -(soccerGlobalY - selfPosition.y);
				soccerGlobalTheta = Angle_Adjustment(atan2(distanceY, distanceX) * 180 / PI);
				soccerLocalTheta = soccerGlobalTheta - selfPosition.dir;
				soccerLocalX = -(sqrt(pow(distanceY,2)+pow(distanceX,2)) * sin(soccerLocalTheta * DEG2RAD));
				soccerLocalY = sqrt(pow(distanceY,2)+pow(distanceX,2)) * cos(soccerLocalTheta * DEG2RAD);
			}
			else
			{
				soccerGlobalX = soccerTmpX;
				soccerGlobalY = soccerTmpY;
				soccerGlobalTheta = soccerTmpTheta;
				soccerLocalX = soccer.x_dis;
				soccerLocalY = soccer.y_dis;
				soccerLocalTheta = soccer.theta;
			}
		}
		else if(localizationPosition.weight < lowThreshold)
		{
			ROS_INFO("SoccerPosition Section 3; Weight under lowThreshold");
			soccerGlobalX = 0.0;
			soccerGlobalY = 0.0;
			weight = 0.0;
			weightCnt = 0;

			for(std::map<std::string, CharacterInfo*>::iterator it = robotCupInfo->characterInfo->who.begin(); it != robotCupInfo->characterInfo->who.end(); it++)
			{
				if(it->first != itMyself->first && it->first != itMyself->second->which_robot)
				{
					if(it->second->object["soccer"].exist_flag == true && it->second->weight >= lowThreshold)
					{
						soccerGlobalX += (it->second->object["soccer"].global.x_pos * it->second->weight);
						soccerGlobalY += (it->second->object["soccer"].global.y_pos * it->second->weight);
						weight += it->second->weight;
						weightCnt++;
					}
				}
			}

			if(weightCnt >= 1)
			{
				soccerGlobalX /= weight;
				soccerGlobalY /= weight;
				distanceX = soccerGlobalX - selfPosition.x;
				distanceY = -(soccerGlobalY - selfPosition.y);
				soccerGlobalTheta = Angle_Adjustment(atan2(distanceY, distanceX) * 180 / PI);
				soccerLocalTheta = soccerGlobalTheta - selfPosition.dir;
				soccerLocalX = -(sqrt(pow(distanceY,2)+pow(distanceX,2)) * sin(soccerLocalTheta * DEG2RAD));
				soccerLocalY = sqrt(pow(distanceY,2)+pow(distanceX,2)) * cos(soccerLocalTheta * DEG2RAD);
			}
			else
			{
				soccerGlobalX = soccerTmpX;
				soccerGlobalY = soccerTmpY;
				soccerGlobalTheta = soccerTmpTheta;
				soccerLocalX = soccer.x_dis;
				soccerLocalY = soccer.y_dis;
				soccerLocalTheta = soccer.theta;
			}
		}
	}
	else
	{
		ROS_INFO("SoccerPosition Section 4; No Soccer");
		soccerGlobalX = 0.0;
		soccerGlobalY = 0.0;
		weight = 0.0;
		weightCnt = 0;

		for(std::map<std::string, CharacterInfo*>::iterator it = robotCupInfo->characterInfo->who.begin(); it != robotCupInfo->characterInfo->who.end(); it++)
		{
			if(it->first != itMyself->first && it->first != itMyself->second->which_robot)
			{
				if(it->second->object["soccer"].exist_flag == true && it->second->weight >= lowThreshold)
				{
					soccerGlobalX += (it->second->object["soccer"].global.x_pos * it->second->weight);
					soccerGlobalY += (it->second->object["soccer"].global.y_pos * it->second->weight);
					weight += it->second->weight;
					weightCnt++;
				}
			}
		}

		if(weightCnt >= 1)
		{
			soccerGlobalX /= weight;
			soccerGlobalY /= weight;
			distanceX = soccerGlobalX - selfPosition.x;
			distanceY = -(soccerGlobalY - selfPosition.y);
			soccerGlobalTheta = Angle_Adjustment(atan2(distanceY, distanceX) * 180 / PI);
			soccerLocalTheta = soccerGlobalTheta - selfPosition.dir;
			soccerLocalX = -(sqrt(pow(distanceY,2)+pow(distanceX,2)) * sin(soccerLocalTheta * DEG2RAD));
			soccerLocalY = sqrt(pow(distanceY,2)+pow(distanceX,2)) * cos(soccerLocalTheta * DEG2RAD);
			soccer.existFlag = true;
		}
		else
		{
			soccer.existFlag = itMyself->second->object["soccer"].exist_flag;
			soccerGlobalX = itMyself->second->object["soccer"].global.x_pos;
			soccerGlobalY = itMyself->second->object["soccer"].global.y_pos;
			soccerGlobalTheta = itMyself->second->object["soccer"].global.theta;
			soccerLocalX = itMyself->second->object["soccer"].local.x_pos;
			soccerLocalY = itMyself->second->object["soccer"].local.y_pos;
			soccerLocalTheta = itMyself->second->object["soccer"].local.theta;
		}
	}

	itMyself->second->object["soccer"].exist_flag = soccer.existFlag;
	itMyself->second->object["soccer"].global.x_pos = (int)soccerGlobalX;
	itMyself->second->object["soccer"].global.y_pos = (int)soccerGlobalY;
	itMyself->second->object["soccer"].global.theta = soccerGlobalTheta;
	itMyself->second->object["soccer"].local.x_pos = (int)soccerLocalX;
	itMyself->second->object["soccer"].local.y_pos = (int)soccerLocalY;
	itMyself->second->object["soccer"].local.theta = soccerLocalTheta;
}

void MultiRobotLocalization::calculatePartnerPosition()
{
	bool partnerExistFlag;
	int partnerCnt;
	int weightCnt;
	int distanceX;
	int distanceY;
	float partnerTmpX;
	float partnerTmpY;
	float partnerTmpTheta;
	float partnerGlobalX;
	float partnerGlobalY;
	float partnerGlobalTheta;
	float partnerLocalX;
	float partnerLocalY;
	float partnerLocalTheta;
	float partnerWeight;
	float weight;
	std::string partnerRobot;
	
	if(itMyself->second->which_robot == "robot1")
	{
		partnerRobot = "robot2";
	}
	else if(itMyself->second->which_robot == "robot2")
	{
		partnerRobot = "robot1";
	}
	else
	{
		ROS_ERROR("ObjectPosition, No this robot!!");
	}
	
	if(partner.existFlag)
	{
		partnerTmpTheta = Angle_Adjustment(selfPosition.dir + partner.theta);
		partnerTmpX = selfPosition.x + partner.dis * cos(partnerTmpTheta * DEG2RAD);
		partnerTmpY = selfPosition.y - partner.dis * sin(partnerTmpTheta * DEG2RAD);

		if(localizationPosition.weight >= highThreshold)
		{
			ROS_INFO("PartnerPosition Section 1; Weight above highThreshold");
			partnerGlobalX = partnerTmpX;
			partnerGlobalY = partnerTmpY;
			partnerGlobalTheta = partnerTmpTheta;
			partnerLocalX = partner.x_dis;
			partnerLocalY = partner.y_dis;
			partnerLocalTheta = partner.theta;
		}
		else if(localizationPosition.weight < highThreshold && localizationPosition.weight >= lowThreshold)
		{
			ROS_INFO("PartnerPosition Section 2; Weight under highThreshold and above lowThreshold");
			partnerGlobalX = partnerTmpX * localizationPosition.weight;
			partnerGlobalY = partnerTmpY * localizationPosition.weight;
			weight = localizationPosition.weight;
			weightCnt = 1;

			for(std::map<std::string, CharacterInfo*>::iterator it = robotCupInfo->characterInfo->who.begin(); it != robotCupInfo->characterInfo->who.end(); it++)
			{
				if(it->first != itMyself->first && it->first != itMyself->second->which_robot)
				{
					if(it->second->partner[partnerRobot].exist_flag == true && it->second->weight >= localizationPosition.weight)
					{
						partnerGlobalX += (it->second->partner[partnerRobot].global.x_pos * it->second->weight);
						partnerGlobalY += (it->second->partner[partnerRobot].global.y_pos * it->second->weight);
						weight += it->second->weight;
						weightCnt++;
					}
				}
			}

			if(weightCnt >= 2)
			{
				partnerGlobalX /= weight;
				partnerGlobalY /= weight;
				distanceX = partnerGlobalX - selfPosition.x;
				distanceY = -(partnerGlobalY - selfPosition.y);
				partnerGlobalTheta = Angle_Adjustment(atan2(distanceY, distanceX) * 180 / PI);
				partnerLocalTheta = partnerGlobalTheta - selfPosition.dir;
				partnerLocalX = -(sqrt(pow(distanceY,2)+pow(distanceX,2)) * sin(partnerLocalTheta * DEG2RAD));
				partnerLocalY = sqrt(pow(distanceY,2)+pow(distanceX,2)) * cos(partnerLocalTheta * DEG2RAD);
			}
			else
			{
				partnerGlobalX = partnerTmpX;
				partnerGlobalY = partnerTmpY;
				partnerGlobalTheta = partnerTmpTheta;
				partnerLocalX = partner.x_dis;
				partnerLocalY = partner.y_dis;
				partnerLocalTheta = partner.theta;
			}
		}
		else if(localizationPosition.weight < lowThreshold)
		{
			ROS_INFO("PartnerPosition Section 3; Weight under lowThreshold");
			partnerGlobalX = 0.0;
			partnerGlobalY = 0.0;
			weight = 0.0;
			weightCnt = 0;

			for(std::map<std::string, CharacterInfo*>::iterator it = robotCupInfo->characterInfo->who.begin(); it != robotCupInfo->characterInfo->who.end(); it++)
			{
				if(it->first != itMyself->first && it->first != itMyself->second->which_robot)
				{
					if(it->second->partner[partnerRobot].exist_flag == true && it->second->weight >= lowThreshold)
					{
						partnerGlobalX += (it->second->partner[partnerRobot].global.x_pos * it->second->weight);
						partnerGlobalY += (it->second->partner[partnerRobot].global.y_pos * it->second->weight);
						weight += it->second->weight;
						weightCnt++;
					}
				}
			}

			if(weightCnt >= 1)
			{
				partnerGlobalX /= weight;
				partnerGlobalY /= weight;
				distanceX = partnerGlobalX - selfPosition.x;
				distanceY = -(partnerGlobalY - selfPosition.y);
				partnerGlobalTheta = Angle_Adjustment(atan2(distanceY, distanceX) * 180 / PI);
				partnerLocalTheta = partnerGlobalTheta - selfPosition.dir;
				partnerLocalX = -(sqrt(pow(distanceY,2)+pow(distanceX,2)) * sin(partnerLocalTheta * DEG2RAD));
				partnerLocalY = sqrt(pow(distanceY,2)+pow(distanceX,2)) * cos(partnerLocalTheta * DEG2RAD);
			}
			else
			{
				partnerGlobalX = partnerTmpX;
				partnerGlobalY = partnerTmpY;
				partnerGlobalTheta = partnerTmpTheta;
				partnerLocalX = partner.x_dis;
				partnerLocalY = partner.y_dis;
				partnerLocalTheta = partner.theta;
			}
		}
	}
	else
	{
		ROS_INFO("PartnerPosition Section 4; No partner");
		partnerGlobalX = 0.0;
		partnerGlobalY = 0.0;
		weight = 0.0;
		weightCnt = 0;

		for(std::map<std::string, CharacterInfo*>::iterator it = robotCupInfo->characterInfo->who.begin(); it != robotCupInfo->characterInfo->who.end(); it++)
		{
			if(it->first != itMyself->first && it->first != itMyself->second->which_robot)
			{
				if(it->second->partner[partnerRobot].exist_flag == true && it->second->weight >= lowThreshold)
				{
					partnerGlobalX += (it->second->partner[partnerRobot].global.x_pos * it->second->weight);
					partnerGlobalY += (it->second->partner[partnerRobot].global.y_pos * it->second->weight);
					weight += it->second->weight;
					weightCnt++;
				}
			}
		}

		if(weightCnt >= 1)
		{
			partnerGlobalX /= weight;
			partnerGlobalY /= weight;
			distanceX = partnerGlobalX - selfPosition.x;
			distanceY = -(partnerGlobalY - selfPosition.y);
			partnerGlobalTheta = Angle_Adjustment(atan2(distanceY, distanceX) * 180 / PI);
			partnerLocalTheta = partnerGlobalTheta - selfPosition.dir;
			partnerLocalX = -(sqrt(pow(distanceY,2)+pow(distanceX,2)) * sin(partnerLocalTheta * DEG2RAD));
			partnerLocalY = sqrt(pow(distanceY,2)+pow(distanceX,2)) * cos(partnerLocalTheta * DEG2RAD);
			partner.existFlag = true;
		}
		else
		{
			partner.existFlag = itMyself->second->partner[partnerRobot].exist_flag;
			partnerGlobalX = itMyself->second->partner[partnerRobot].global.x_pos;
			partnerGlobalY = itMyself->second->partner[partnerRobot].global.y_pos;
			partnerGlobalTheta = itMyself->second->partner[partnerRobot].global.theta;
			partnerLocalX = itMyself->second->partner[partnerRobot].local.x_pos;
			partnerLocalY = itMyself->second->partner[partnerRobot].local.y_pos;
			partnerLocalTheta = itMyself->second->partner[partnerRobot].local.theta;
		}
	}

	itMyself->second->partner[partnerRobot].exist_flag = partner.existFlag;
	itMyself->second->partner[partnerRobot].global.x_pos = (int)partnerGlobalX;
	itMyself->second->partner[partnerRobot].global.y_pos = (int)partnerGlobalY;
	itMyself->second->partner[partnerRobot].global.theta = partnerGlobalTheta;
	itMyself->second->partner[partnerRobot].local.x_pos = (int)partnerLocalX;
	itMyself->second->partner[partnerRobot].local.y_pos = (int)partnerLocalY;
	itMyself->second->partner[partnerRobot].local.theta = partnerLocalTheta;
}

void MultiRobotLocalization::getObjectInfo()
{
	soccerDataInitialize();
	goalDataInitialize();
	
	ROS_INFO("objectInfo size = %d", strategy_info->objectInfo.size());
	if(0 < strategy_info->objectInfo.size())
	{
		for(int i = 0; i < strategy_info->objectInfo.size(); i++)
		{
			if(strategy_info->objectInfo[i].object_mode == ObjectMode::SOCCER)
			{
				soccer.width = strategy_info->objectInfo[i].width;
				soccer.height = strategy_info->objectInfo[i].height;
				soccer.size = soccer.width * soccer.height;
				soccer.x = strategy_info->objectInfo[i].x + (soccer.width / 2);
				soccer.y = strategy_info->objectInfo[i].y + (soccer.height / 2);
				soccer.x_dis = strategy_info->objectInfo[i].x_distance;
				soccer.y_dis = strategy_info->objectInfo[i].y_distance;
				soccer.dis = strategy_info->objectInfo[i].distance;
				soccer.theta = -(atan2(soccer.x_dis, soccer.y_dis) * 180 / PI);
				soccer.existFlag = true;
			}
			else if(strategy_info->objectInfo[i].object_mode == ObjectMode::GOAL)
			{
				if(goal.cnt == 0)
				{
					goal.width[0] = strategy_info->objectInfo[i].width;
					goal.height[0] = strategy_info->objectInfo[i].height;
					goal.x[0] = strategy_info->objectInfo[i].x + (goal.width[0] / 2);
					goal.y[0] = strategy_info->objectInfo[i].y + (goal.height[0] / 2);
					goal.cnt = 1;
					goal.existFlag = true;
				}
				else if(goal.cnt == 1)
				{
					goal.width[1] = strategy_info->objectInfo[i].width;
					goal.height[1] = strategy_info->objectInfo[i].height;
					goal.x[1] = strategy_info->objectInfo[i].x + (goal.width[1] / 2);
					goal.y[1] = strategy_info->objectInfo[i].y + (goal.height[1] / 2);
					goal.cnt = 2;
					goal.existFlag = true;
					// ROS_INFO("i n 2");
				}
			}
			else if(strategy_info->objectInfo[i].object_mode == ObjectMode::PARTNER)
			{
				partner.width = strategy_info->objectInfo[i].width;
				partner.height = strategy_info->objectInfo[i].height;
				partner.size = partner.width * partner.height;
				partner.x = strategy_info->objectInfo[i].x + (partner.width / 2);
				partner.y = strategy_info->objectInfo[i].y + (partner.height / 2);
				partner.x_dis = strategy_info->objectInfo[i].x_distance;
				partner.y_dis = strategy_info->objectInfo[i].y_distance;
				partner.dis = strategy_info->objectInfo[i].distance;
				partner.theta = -(atan2(partner.x_dis, partner.y_dis) * 180 / PI);
				partner.existFlag = true;
			}
			else if(strategy_info->objectInfo[i].object_mode == ObjectMode::ENEMY)
			{
				enemy.width = strategy_info->objectInfo[i].width;
				enemy.height = strategy_info->objectInfo[i].height;
				enemy.size = enemy.width * enemy.height;
				enemy.x = strategy_info->objectInfo[i].x + (enemy.width / 2);
				enemy.y = strategy_info->objectInfo[i].y + (enemy.height / 2);
				enemy.x_dis = strategy_info->objectInfo[i].x_distance;
				enemy.y_dis = strategy_info->objectInfo[i].y_distance;
				enemy.dis = strategy_info->objectInfo[i].distance;
				enemy.theta = -(atan2(enemy.x_dis, enemy.y_dis) * 180 / PI);
				enemy.existFlag = true;
			}
			else if(strategy_info->objectInfo[i].object_mode == ObjectMode::NOTHING)
			{
				
			}
		}
		ROS_INFO("soccer size = %d", soccer.size);
	}
	else
	{
		ROS_INFO("No object");
	}

	ROS_INFO("soccer.x_dis: %d", soccer.x_dis);
	ROS_INFO("soccer.y_dis: %d", soccer.y_dis);
	ROS_INFO("soccer.theta: %f\n", soccer.theta);

	strategy_info->objectInfo.clear();
}

float MultiRobotLocalization::Angle_Adjustment(float angle)
{
    if(angle < 0.0)
	{
        return angle + 360.0;
	}
    else if(angle >= 360.0)
	{
        return angle - 360.0;
	}
    else
	{
        return angle;
	}
}

void MultiRobotLocalization::transmitRoboCupInfo()
{
	ros2MultiCom->sendRobotCupInfo(robotCupInfo);
	robotCupInfo->characterInfo->testShow();
	robotCupInfo->characterInfo->testShowTimer();
	robotCupInfo->characterInfo->setTimerPass(1000, false);
	saveData();
	ROS_INFO("PRS = %s", robotCupInfo->characterInfo->getPRS().c_str());
}

void MultiRobotLocalization::saveData()
{
	fstream fp;
    fp.open(filePath.c_str(), std::ios::out | std::ios::app);

	fp << std::to_string(loopTimes) + "\t";
	fp << std::to_string(itMyself->second->weight) + "\t";
	fp << std::to_string(robotCupInfo->characterInfo->callBackTimer[itMyself->second->which_robot].getTimeMs()) + "\n";

	fp.close();
}

void MultiRobotLocalization::publishPosition()
{
	tku_msgs::RobotPos robotPosition;
	tku_msgs::SoccerPos soccerPosition;

	for(int i = 0; i < 4; i++)
	{
		robotPosition.number = i;
		robotPosition.x = robotCupInfo->characterInfo->who[StrE::robot[i]]->global.x_pos;
		robotPosition.y = robotCupInfo->characterInfo->who[StrE::robot[i]]->global.y_pos;
		robotPosition.dir = robotCupInfo->characterInfo->who[StrE::robot[i]]->global.theta;
		RobotPos_Publisher.publish(robotPosition);
	}

	soccerPosition.x = itMyself->second->object["soccer"].global.x_pos;
	soccerPosition.y = itMyself->second->object["soccer"].global.y_pos;
	SoccerPos_Publisher.publish(soccerPosition);
}

void MultiRobotLocalization::getLocalizationPositionFunction(const tku_msgs::LocalizationPos &msg)
{
	localizationPosition.position.x = msg.x;
    localizationPosition.position.y = msg.y;
    localizationPosition.theta = msg.dir;
	localizationPosition.weight = msg.weight;
}

void MultiRobotLocalization::robotPositionDataInitialize()
{
	localizationPosition.position.x = 0;
	localizationPosition.position.y = 0;
	localizationPosition.theta = 0.0;
	localizationPosition.weight = 0.0;
}

void MultiRobotLocalization::soccerDataInitialize()
{
	soccer.existFlag = false;
	// soccer.width = 0;
	// soccer.height = 0;
	// soccer.size = 0;
	// soccer.x = 0;
	// soccer.y = 0;
	// soccer.x_dis = 0;
	// soccer.y_dis = 0;
	// soccer.theta = 0.0;
}

void MultiRobotLocalization::partnerDataInitialize()
{
	partner.existFlag = false;
	// partner.width = 0;
	// partner.height = 0;
	// partner.size = 0;
	// partner.x = 0;
	// partner.y = 0;
	// partner.x_dis = 0;
	// partner.y_dis = 0;
	// partner.theta = 0.0;
}

void MultiRobotLocalization::enemyDataInitialize()
{
	enemy.existFlag = false;
	// enemy.width = 0;
	// enemy.height = 0;
	// enemy.size = 0;
	// enemy.x = 0;
	// enemy.y = 0;
	// enemy.x_dis = 0;
	// enemy.y_dis = 0;
	// enemy.theta = 0.0;
}

void MultiRobotLocalization::goalDataInitialize()
{
	goal.existFlag = false;
	for(int i = 0; i < (sizeof(goal.x)/sizeof(goal.x[0])); i++)
	{
		goal.x[i] = 0;
	}
	for(int i = 0; i < (sizeof(goal.y)/sizeof(goal.y[0])); i++)
	{
		goal.y[i] = 0;
	}
	for(int i = 0; i < (sizeof(goal.width)/sizeof(goal.width[0])); i++)
	{
		goal.width[i] = 0;
	}
	for(int i = 0; i < (sizeof(goal.height)/sizeof(goal.height[0])); i++)
	{
		goal.height[i] = 0;
	}
	for(int i = 0; i < (sizeof(goal.size)/sizeof(goal.size[0])); i++)
	{
		goal.size[i] = 0;
	}
	goal.cnt = 0;
}

void MultiRobotLocalization::saveDataInitialize()
{
	fstream fp;
    fp.open(filePath.c_str(), std::ios::out | std::ios::trunc);

	fp << "loopTimes\t";
	fp << "callBackTimer\t";
	fp << "weight\n";

	fp.close();
}

MultiRobotLocalization::MultiRobotLocalization(ros::NodeHandle &nh)
{
	strategy_info = StrategyInfoInstance::getInstance();
	tool = ToolInstance::getInstance();
	ros_com = RosCommunicationInstance::getInstance();
	walk_con = WalkContinuouseInstance::getInstance();
	ros2MultiCom = Ros2MultiCommunication::getInstance();
	robotCupInfo = RobotCupInfo::getInstance();
	client.start();

	RobotPos_Publisher = nh.advertise<tku_msgs::RobotPos>("/localization/robotpos", 1000);
	SoccerPos_Publisher = nh.advertise<tku_msgs::SoccerPos>("/localization/soccerpos", 1000);
	
	localizationPos_Subscriber = nh.subscribe("/localization/localizationpos", 1, &MultiRobotLocalization::getLocalizationPositionFunction, this);

	selfPosition.x = 0;
	selfPosition.y = 0;
	selfPosition.dir = 0;

	robotPositionDataInitialize();

	soccerDataInitialize();

	partnerDataInitialize();

	enemyDataInitialize();
	
	goalDataInitialize();

	filePath = ros::package::getPath("localization") + "/Parameter/SaveData.csv";
	saveDataInitialize();

	itMyself = robotCupInfo->characterInfo->who.find("myself");

	tool->Delay(500);
    ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2048, 200);
    tool->Delay(50);
    ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1700, 200);
    tool->Delay(50);
    ros_com->sendSensorReset();//IMU值重製
}

MultiRobotLocalization::~MultiRobotLocalization()
{
	StrategyInfoInstance::deleteInstance();
	ToolInstance::deleteInstance();
	RosCommunicationInstance::deleteInstance();
	WalkContinuouseInstance::deleteInstance();
	Ros2MultiCommunication::deleteInstance();
	RobotCupInfo::deleteInstance();
}
