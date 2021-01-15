#include <LocalizationBase/LocalizationBase.h>

LocalizationBase::LocalizationBase()
{

}
LocalizationBase::~LocalizationBase()
{
    
}

float LocalizationBase::Angle_Adjustment(float angle)
{
    if (angle < 0.0)
        return angle + 360.0;
    else if (angle >= 360.0)
        return angle - 360.0;
    else
        return angle;
}

void LocalizationBase::AngleLUT()
{
    double ang_PI;
    for (int ang = 0; ang <= 360; ang++)
    {
        ang_PI = ang * DEG2RAD;
        Angle_sin.push_back(sin(ang_PI));
        Angle_cos.push_back(cos(ang_PI));
    }
}

int LocalizationBase::Frame_Area(int coordinate, int range)
{
    if (coordinate < 0)
        coordinate = 0;
    else if (coordinate >= range)
        coordinate = range - 1;
    return coordinate;
}

double LocalizationBase::Slope(Vec4i line)
{
    if((line[2]-line[0]) == 0) return 90;
    else {
        double s = atan2((double(line[3])-double(line[1])),(double(line[2])-double(line[0])))*RAD2DEG;
        if(s > 90) return -180+s;
        else if(s < -90) return 180+s;
        else return s;
    }
}