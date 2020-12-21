#include <ros/ros.h>
#include <random>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <highgui.h>
#include <sys/time.h>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <vector>

#define MAP_LENGTH      1100
#define MAP_WIDTH       800
#define MAP_MAX_LENGTH  1360        //sqrt((pow(MAP_LENGTH,2)+pow(MAP_WIDTH,2)))
#define MOTORDEG        0.08789     //360/4096 (deg)
#define DEG2RAD         M_PI/180
#define RAD2DEG         180/M_PI
#define PI              3.14159265
#define PARTICLNUM      1000
#define EXCELLENTPARTICLNUM      20

using namespace std;
using namespace cv;

struct coordinate
{
    int X;
    int Y;
};

struct pointdata
{
    coordinate pos;
    int theta;
};

struct featuredata
{
    int X;
    int Y;
    int x_dis;
    int y_dis;
    int dis;
};

struct  motordata
{
    int pos;
    int speed;
};

struct Distance
{
    int x_dis;
    int y_dis;
    int dis;
};

struct scan_line
{
    vector<featuredata> feature_point;
};

struct ParticlePoint
{
    coordinate postion;
    coordinate FOV_Bottom_Right;
    coordinate FOV_Bottom_Left;
    coordinate FOV_Top_Right;
    coordinate FOV_Top_Left;
    
    int fitness_value;
    float weight;
    int particle_num;
    float angle;
    float FOV_dir;

    vector<scan_line> featurepoint_scan_line;

    bool operator >(const ParticlePoint& rhs) const   //vector 使用struct型態 針對其中成員排序使用
	{
		return weight > rhs.weight;
	}
};

class LocalizationBase
{
    protected:
        vector<double> Angle_sin;
        vector<double> Angle_cos;
    public:
        cv::Mat Soccer_Field;
    public:
        LocalizationBase();
        ~LocalizationBase();

        float Angle_Adjustment(float angle);
        void AngleLUT();
        int Frame_Area(int coordinate, int range);
};