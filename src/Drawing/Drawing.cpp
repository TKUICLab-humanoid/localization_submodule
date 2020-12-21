#include <Drawing/Drawing.h>

Drawing::Drawing()
{

}
Drawing::~Drawing()
{

}

Mat Drawing::convertTo3Channels(const Mat &binImg)
{
    Mat three_channel = Mat::zeros(binImg.rows, binImg.cols, CV_8UC3);
    vector<Mat> channels;
    for (int i = 0; i < 3; i++)
    {
        channels.push_back(binImg);
    }
    merge(channels, three_channel);
    return three_channel;
}

/*Mat Drawing::DrawField()   //RoboCup field
{
    Mat Field(MAP_WIDTH, MAP_LENGTH, CV_8UC3, Scalar(0, 0, 0));
    Mat Field_edge;

    rectangle(Field, Point(102,102), Point(998,698), Scalar(255, 255, 255), 3);
    rectangle(Field, Point(102,152), Point(298,648), Scalar(255, 255, 255), 3);
    rectangle(Field, Point(802,152), Point(998,648), Scalar(255, 255, 255), 3);
    rectangle(Field, Point(102,252), Point(198,548), Scalar(255, 255, 255), 3);
    rectangle(Field, Point(902,252), Point(998,548), Scalar(255, 255, 255), 3);

    circle(Field, Point(550,400), 73, Scalar(255, 255, 255), 3);

    line(Field, Point(550,102), Point(550,698), Scalar(255, 255, 255), 3);

    line(Field, Point(542,400), Point(558,400), Scalar(255, 255, 255), 3);
    
    line(Field, Point(250,392), Point(250,408), Scalar(255, 255, 255), 3);
    line(Field, Point(242,400), Point(258,400), Scalar(255, 255, 255), 3);

    line(Field, Point(850,392), Point(850,408), Scalar(255, 255, 255), 3);
    line(Field, Point(842,400), Point(858,400), Scalar(255, 255, 255), 3);

    cv::Canny(Field, Field_edge, 50, 150, 3);
    Field_edge=convertTo3Channels(Field_edge);
    line(Field_edge, Point(42,268), Point(102,268), Scalar(0, 255, 255), 3);
    line(Field_edge, Point(42,268), Point(42,532), Scalar(0, 255, 255), 3);
    line(Field_edge, Point(42,532), Point(102,532), Scalar(0, 255, 255), 3);
    line(Field_edge, Point(998,268), Point(1058,268), Scalar(0, 255, 255), 3);
    line(Field_edge, Point(1058,268), Point(1058,532), Scalar(0, 255, 255), 3);
    line(Field_edge, Point(998,532), Point(1058,532), Scalar(0, 255, 255), 3);
    return Field_edge;
}*/

Mat Drawing::DrawField()    //E223 field
{
    Mat Field(MAP_WIDTH, MAP_LENGTH, CV_8UC3, Scalar(0, 0, 0));
    Mat Field_edge;

    rectangle(Field, Point(102,122), Point(998,678), Scalar(255, 255, 255), 3);
    rectangle(Field, Point(102,172), Point(298,628), Scalar(255, 255, 255), 3);
    rectangle(Field, Point(802,172), Point(998,628), Scalar(255, 255, 255), 3);
    rectangle(Field, Point(102,252), Point(198,548), Scalar(255, 255, 255), 3);
    rectangle(Field, Point(902,252), Point(998,548), Scalar(255, 255, 255), 3);

    circle(Field, Point(550,400), 73, Scalar(255, 255, 255), 3);

    line(Field, Point(550,122), Point(550,678), Scalar(255, 255, 255), 3);

    line(Field, Point(542,400), Point(558,400), Scalar(255, 255, 255), 3);
    
    line(Field, Point(250,392), Point(250,408), Scalar(255, 255, 255), 3);
    line(Field, Point(242,400), Point(258,400), Scalar(255, 255, 255), 3);

    line(Field, Point(850,392), Point(850,408), Scalar(255, 255, 255), 3);
    line(Field, Point(842,400), Point(858,400), Scalar(255, 255, 255), 3);

    cv::Canny(Field, Field_edge, 50, 150, 3);
    Field_edge=convertTo3Channels(Field_edge);
    line(Field_edge, Point(42,268), Point(102,268), Scalar(0, 255, 255), 3);
    line(Field_edge, Point(42,268), Point(42,532), Scalar(0, 255, 255), 3);
    line(Field_edge, Point(42,532), Point(102,532), Scalar(0, 255, 255), 3);
    line(Field_edge, Point(998,268), Point(1058,268), Scalar(0, 255, 255), 3);
    line(Field_edge, Point(1058,268), Point(1058,532), Scalar(0, 255, 255), 3);
    line(Field_edge, Point(998,532), Point(1058,532), Scalar(0, 255, 255), 3);
    return Field_edge;
}

Mat Drawing::DrawFOV()
{
    Mat tmp_FOV = Soccer_Field.clone();
    for(int i = 0; i < particlepoint.size(); i++)
    {
        FOV_Point_tmp.push_back(Point(particlepoint[i].FOV_Bottom_Right.X,particlepoint[i].FOV_Bottom_Right.Y));
        FOV_Point_tmp.push_back(Point(particlepoint[i].FOV_Top_Right.X,particlepoint[i].FOV_Top_Right.Y));
        FOV_Point_tmp.push_back(Point(particlepoint[i].FOV_Top_Left.X,particlepoint[i].FOV_Top_Left.Y));
        FOV_Point_tmp.push_back(Point(particlepoint[i].FOV_Bottom_Left.X,particlepoint[i].FOV_Bottom_Left.Y));

        //const Point* ppt[1] = {&tmp[0]};
        //int npt[] = {4};

        cv::polylines(tmp_FOV, FOV_Point_tmp, true, Scalar(128, 128, 128), 1);//第2個引數可以採用contour或者contours，均可
	    //cv::fillPoly(tmp_FOV, ppt, npt,  1, Scalar(128, 128, 128));//fillPoly函式的第二個引數是二維陣列！！

        circle(tmp_FOV, Point(particlepoint[i].postion.X,particlepoint[i].postion.Y), 2, Scalar(255, 255, 0), 4);
    }
    FOV_Point_tmp.clear();
    return tmp_FOV;
}

void Drawing::DrawFeaturePoint(Mat* iframe)
{
    //Mat oframe = FOV_Field.clone();
    /*for(int i = 0; i < particlepoint.size(); i++)
    {
        for(int j = 0; j < particlepoint[i].featurepoint.size(); j++)
        {
            if(particlepoint[i].featurepoint[j].X != 0 && particlepoint[i].featurepoint[j].Y !=0)
            {
                circle(*iframe, Point(particlepoint[i].featurepoint[j].X,particlepoint[i].featurepoint[j].Y), 3, Scalar(255, 0, 0), 1);
            }
        }
    }*/
    //return *iframe;
}

Mat Drawing::DrawParticlePoint()
{
    Mat ParticlePoint_tmp = Soccer_Field.clone();
    for(int i = 0; i < particlepoint.size(); i++)
    {
        circle(ParticlePoint_tmp, Point(particlepoint[i].postion.X,particlepoint[i].postion.Y), 3, Scalar(255, 0, 0), 1);
    }
    return ParticlePoint_tmp;
}

Mat Drawing::DrawRobotPos()
{
    Mat oframe = Soccer_Field.clone();

    Robot_FOV_tmp.push_back(Point(Robot_Position.FOV_Bottom_Right.X,Robot_Position.FOV_Bottom_Right.Y));
    Robot_FOV_tmp.push_back(Point(Robot_Position.FOV_Top_Right.X,Robot_Position.FOV_Top_Right.Y));
    Robot_FOV_tmp.push_back(Point(Robot_Position.FOV_Top_Left.X,Robot_Position.FOV_Top_Left.Y));
    Robot_FOV_tmp.push_back(Point(Robot_Position.FOV_Bottom_Left.X,Robot_Position.FOV_Bottom_Left.Y));

    cv::polylines(oframe, Robot_FOV_tmp, true, Scalar(128, 128, 128), 1);//第2個引數可以採用contour或者contours，均可

    circle(oframe, Point(Robot_Position.postion.X,Robot_Position.postion.Y), 5, Scalar(255, 255, 0), 2);
    for(int i = 0; i < Robot_Position.featurepoint_scan_line.size(); i++)
    {
        for(int j = 0; j < Robot_Position.featurepoint_scan_line[i].feature_point.size(); j++)
        {
            if(Robot_Position.featurepoint_scan_line[i].feature_point[j].X != -1 && Robot_Position.featurepoint_scan_line[i].feature_point[j].Y != -1)
            {
                circle(oframe, Point(Robot_Position.featurepoint_scan_line[i].feature_point[j].X,Robot_Position.featurepoint_scan_line[i].feature_point[j].Y), 3, Scalar(255, 0, 0), 3);
            }
        }
    }

    Robot_FOV_tmp.clear();
    return oframe;
}
