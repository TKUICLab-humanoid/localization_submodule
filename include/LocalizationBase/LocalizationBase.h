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
#include <eigen3/Eigen/Dense>

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
using Eigen::MatrixXd;
using Eigen::VectorXd;

enum class LineType
{
    Horizontal,
    Vertical,
    Slope
};

struct FieldLine_data
{
    LineType linetype;
    float para_a; // y=ax+b
    float para_b;
    float x; // when x = value
    Point start_point;
    Point end_point;
    int Line_ID;
};

struct movement_data
{
    float  straight;
    float  drift;
    float  rotational;
    float  moving;
    float  dt;

};

struct IMU_data
{
    float  roll;
    float  pitch;
    float  yaw;

};

struct Pointdata
{
    Point pose;
    double angle;
};

struct featuredata
{
    int x;
    int y;
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

struct LineINF
{
    Point end_point;
    Point start_point;
    Point center_point;
    double Line_length;
    double Line_theta;  
    double distance;
    Point Nearest_point;
    Eigen::Vector2d mu;
    bool obersvated;
    Eigen::Matrix2d sigma;
    bool update;

};

struct all_linedata
{
    vector<LineINF> Lineinformation;
};

static bool tocompare(Vec4i &s1, Vec4i &s2){

   return s1[0] > s2[0];
}

struct ParticlePoint
{
    // FOV
    // 0:Top_Left 1:Top_right 2:Bottom_Right 3:Bottom_Left
    Point FOV_corrdinate[4]; 
    float FOV_dir;
    Point FOV_Bottom_Right;
    Point FOV_Bottom_Left;
    Point FOV_Top_Right;
    Point FOV_Top_Left;
    
    // position
    Pointdata pos;
    // Point postion;

    // weight
    double weight;
    int fitness_value;
    double likehood;

    // factors
    vector<int> factors[15];

    // wfactors
    double wfactors;

    // observation information
    vector<scan_line> featurepoint_scan_line;
    vector<LineINF> landmark_list;
    
    bool operator >(const ParticlePoint& rhs) const   //vector 使用struct型態 針對其中成員排序使用
	{
		return weight > rhs.weight;
	}

    // not sure  
    int particle_num;
    // double angle;
    
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
        double Slope(Vec4i line);
        void AngleLUT();
        int Frame_Area(int coordinate, int range);
        int Cross(Point A, Point B, Point P);  //點P與線段AB位置關係
        int intersect(Vec4i X, Vec4i Y);
        Point IntersectPoint(Vec4i X, Vec4i Y);
        double dis2(Point a, Point b);                //點a、b距離的平方
        double MinDistance(Vec4i FOVbottom, Point A);
        Point MinIntersectPoint(Vec4i line, Point A, double mindistance);
        float normalize_angle(float phi) ;
        float normalize_angle_RAD(float phi) ;
        LineINF LineInformation(Point A, Point B, Point Bottom_left, Point Bottom_right);
        bool Line_observation_data_flag = true;
        int Line_observation_data_Size ;
        double gauss(double sigma, double mu = 0);

};