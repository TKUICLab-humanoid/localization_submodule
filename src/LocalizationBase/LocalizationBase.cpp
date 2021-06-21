#include <LocalizationBase/LocalizationBase.h>

LocalizationBase::LocalizationBase()
{

}
LocalizationBase::~LocalizationBase()
{
    
}

double LocalizationBase::gauss(double sigma, double mu)
{
    // ROS_INFO("gauss");
    std::random_device rd;
    std::mt19937 generator(rd());
    std::normal_distribution<> dist;
    std::normal_distribution<> norm {mu, sigma};
    return norm(rd);
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

float LocalizationBase::normalize_angle(float phi) 
{
  //Normalize phi to be between -pi and pi
    while(phi > 180.0) {
        phi = phi - 2 * 180.0;
    }

    while(phi < -180.0) {
        phi = phi + 2 * 180.0;
    }
    return phi;
}

float LocalizationBase::normalize_angle_RAD(float phi) 
{
  //Normalize phi to be between -pi and pi
    while(phi > M_PI) {
        phi = phi - 2 * M_PI;
    }

    while(phi < -M_PI) {
        phi = phi + 2 * M_PI;
    }
    return phi;
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

int LocalizationBase::Cross(Point A, Point B, Point P)      //點P與線段AB位置關係
{       
    Point AB = Point(B.x - A.x, B.y - A.y );
	Point AP = Point( P.x - A.x, P.y - A.y );
    double cross = AB.x*AP.y - AB.y*AP.x;
    // double   dot = AB.x*AP.x + AB.y*AP.y;
    // if (cross < 0) return -1;       //逆時針
	// else if (cross > 0) return 1;   //順時針
	// else if (dot < 0) return -2;    //反延長線
	// else if (dot >= 0 && dis2(A,B) >= dis2(A,P))
	// {
	// 	if (dis2(A, B) < dis2(A, P)) return 2;  //延長線
	// 	return 0;                          //在線上
	// }                                      
}

int LocalizationBase::intersect(Vec4i X, Vec4i Y)
{
    Point Xstart = Point(X[0],X[1]);
    Point Xend = Point(X[2],X[3]);
    Point Ystart = Point(Y[0],Y[1]);
    Point Yend = Point(Y[2],Y[3]);

    float c1 = Cross(Xstart,Xend,Ystart);
    float c2 = Cross(Xstart,Xend,Yend);
    float c3 = Cross(Ystart,Yend,Xstart);
    float c4 = Cross(Ystart,Yend,Xend);

    if((c1*c2) < 0 && (c3*c4) < 0)  //兩線段相交, 距離為0
	{
        return 1;
    }else if(c1 == 0 && c2 ==0)
    {
        return 2;
    }
    return 0;
}

Point LocalizationBase::IntersectPoint(Vec4i X, Vec4i Y)
{
    Point intersectPoint = Point(0,0);
    Point Xstart = Point(X[0],X[1]);
    Point Xend = Point(X[2],X[3]);
    Point Ystart = Point(Y[0],Y[1]);
    Point Yend = Point(Y[2],Y[3]);
    float u = ((Yend.y-Ystart.y)*(Xend.x-Xstart.x))-((Yend.x-Ystart.x)*(Xend.y-Xstart.y));
    float a = ((Yend.x-Ystart.x)*(Xstart.y-Ystart.y))-((Yend.y-Ystart.y)*(Xstart.x-Ystart.x));
    float b = ((Xend.x-Xstart.x)*(Xstart.y-Ystart.y))-((Xend.y-Xstart.y)*(Xstart.x-Ystart.x));
    float ua = a/u;
    float ub = b/u;
    
    if(ua >= 0.0 && ua <= 1.0 && ub >= 0.0 && ub <= 1.0f)
    {
        intersectPoint.x = Xstart.x + ua * (Xend.x - Xstart.x);
        intersectPoint.y = Xstart.y + ua * (Xend.y - Xstart.y);
    }
    return intersectPoint;
}

LineINF LocalizationBase::LineInformation(Point A, Point B, Point Bottom_left, Point Bottom_right)
{
    LineINF lineinf;
    Point v1 = Point(B.x-A.x,B.y-A.y);
    Point v2 = Point(Bottom_right.x-Bottom_left.x,Bottom_right.y-Bottom_left.y);
    float l1 = sqrt(dis2(A,B));
    float l2 = sqrt(dis2(Bottom_right,Bottom_left));
    Point Bottom_center = Point((Bottom_right.x+Bottom_left.x)/2,(Bottom_right.y+Bottom_left.y)/2);
    double cross = v1.x * v2.y - v1.y * v2.x;
    Vec4i tmp = {A.x,A.y,B.x,B.y};
    lineinf.start_point = Point(A.x,A.y);
    lineinf.end_point = Point(B.x,B.y);
    lineinf.center_point = Point((A.x+B.x)/2,(A.y+B.y)/2);
    lineinf.Line_length = sqrt(pow((A.x-B.x),2)+pow((A.y-B.y),2));
    lineinf.Line_theta = asin(cross/l1/l2)*RAD2DEG;  //相對於FOVBottom的角度
    double mindis = MinDistance(tmp,Bottom_center/*Point(abs(Bottom_right.x-Bottom_left.x),abs(Bottom_right.y-Bottom_left.y))*/);
    lineinf.distance = mindis;
    lineinf.Nearest_point = MinIntersectPoint(tmp,Bottom_center,mindis);

    // ROS_INFO("lineinf point = %d %d %d %d ",lineinf.start_point.x,lineinf.start_point.y,lineinf.end_point.x,lineinf.end_point.y);
    // ROS_INFO("distance = %f Nearest_point = %d %d",lineinf.distance,lineinf.Nearest_point.x,lineinf.Nearest_point.y);

    return lineinf;
}

double LocalizationBase::dis2(Point a, Point b)                //點a、b距離的平方
{
	return (a.x - b.x)*(a.x - b.x) + (a.y-b.y)*(a.y-b.y);
}

double LocalizationBase::MinDistance(Vec4i Line, Point A)
{
    Point Linestart = Point(Line[0],Line[1]);
    Point Lineend = Point(Line[2],Line[3]);
    double disline2 = dis2(Linestart, Lineend);  
    double r = ((A.x-Linestart.x)*(Lineend.x-Linestart.x) + (A.y-Linestart.y)*(Lineend.y-Linestart.y)) / disline2;

    if (r <= 0) return sqrt(dis2(Linestart, A));
	else if (r >= 1) return sqrt(dis2(Lineend, A));
	else
	{
		double AC = r*sqrt(disline2);
		return sqrt(dis2(Linestart,A)-AC*AC);
	}
}

Point LocalizationBase::MinIntersectPoint(Vec4i line, Point A, double mindistance)//A點與線段最近的點
{
    Point start = Point(line[0],line[1]);
    Point end = Point(line[2],line[3]);
    int x1 = start.x;
    int y1 = start.y;
    int x2 = end.x;
    int y2 = end.y;
    float para_a = (float)(y1-y2)/(x1-x2);
    float para_b = (float)(x1*y2-x2*y1)/(x1-x2);
    Point minIntersectPoint = Point(0,0);
    double min_value = 0.0;
    
    // ROS_INFO("x2-x1 == %d",abs(x2-x1));
    if(abs(x2-x1) == 0)
    {
        for(int i = 0; i < abs(y2-y1); i++ )
        {
            float x = 0.0;
            float y = 0.0;
            double dis = 0.0;
            if(y2<y1)
            {
                y = y2 + i;
            }else{
                y = y1 + i;
            }
            x = x1;
            dis = sqrt(pow(A.x-x,2)+pow(A.y-y,2));
            if( i == 0)
            {
                min_value = dis;
                minIntersectPoint = Point(x,y);
            }
            if(min_value > dis)
            {
                minIntersectPoint = Point(x,y);
                min_value = dis;
            }
            // ROS_INFO(" dis = %f min_value = %f,mindistance = %f",dis ,min_value,mindistance);
        }
    }else{
        for(int i = 0; i < abs(x2-x1); i++ )
        {
            float x = 0.0;
            float y = 0.0;
            double dis = 0.0;
            if(x2<x1)
            {
                x = x2 + i;
            }else{
                x = x1 + i;
            }
            y = x*para_a+para_b;
            dis = sqrt(pow(A.x-x,2)+pow(A.y-y,2));
            if( i == 0)
            {
                min_value = dis;
                minIntersectPoint = Point(x,y);
            }
            if(min_value > dis)
            {
                minIntersectPoint = Point(x,y);
                min_value = dis;
            }
            // ROS_INFO(" dis = %f min_value = %f,mindistance = %f",dis ,min_value,mindistance);
        }
    }

    
    // ROS_INFO("MinIntersectPoint = (%d,%d)",minIntersectPoint.x,minIntersectPoint.y);
    return minIntersectPoint;
}
