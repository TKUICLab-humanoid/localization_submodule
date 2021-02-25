#include <Drawing/Drawing.h>

Drawing::Drawing()
{

}
Drawing::~Drawing()
{

}

Mat Drawing::convertTo3Channels(const Mat &binImg)
{
    ROS_INFO("convertTo3Channels");
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
    ROS_INFO("DrawField");
    Mat Field(MAP_WIDTH, MAP_LENGTH, CV_8UC3, Scalar(0, 0, 0));
    Mat Field_edge;
    
    vector<Point> field_point1 = {Point(102,122),Point(998,122),Point(998,678),Point(102,678)};
    vector<Point> field_point2 = {Point(102,172),Point(298,172),Point(298,628),Point(102,628)};
    vector<Point> field_point3 = {Point(802,172),Point(998,172),Point(998,628),Point(802,628)};
    vector<Point> field_point4 = {Point(102,252),Point(198,252),Point(198,548),Point(102,548)};
    vector<Point> field_point5 = {Point(902,252),Point(998,252),Point(998,548),Point(902,548)};
    vector<Point> field_point6 = {Point(550,122),Point(550,678)};

    vector<vector<Point>> field_point = {field_point1,field_point2,field_point3,field_point4,field_point5,field_point6};

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

    field_list.clear();
    for(int i = 0; i < field_point.size(); i++ )
    {
        vector<Point> field_tmp = field_point[i];
        for(int j = 0; j < field_tmp.size(); j++)
        {
            int x1 = 0;
            int y1 = 0;
            int x2 = 0;
            int y2 = 0;
            // FieldLine_data field_data_tmp;
            if( j < field_tmp.size()-1)
            {
                x1 = field_tmp[j].x;
                y1 = field_tmp[j].y;
                x2 = field_tmp[j+1].x;
                y2 = field_tmp[j+1].y;
            }else{
                x1 = field_tmp[j].x;
                y1 = field_tmp[j].x;
                x2 = field_tmp[0].y;
                y2 = field_tmp[0].y;
            }
            FieldLine_data FieldLine_tmp;
            if((y1-y2) == 0)
            {
                if(x1<x2)
                {
                    FieldLine_tmp.start_point = Point(x1,y1);
                    FieldLine_tmp.end_point = Point(x2,y2);
                }else{
                    FieldLine_tmp.start_point = Point(x2,y2);
                    FieldLine_tmp.end_point = Point(x1,y1);
                }
            }else{
                if(y1<y2)
                {
                    FieldLine_tmp.start_point = Point(x1,y1);
                    FieldLine_tmp.end_point = Point(x2,y2);
                }else{
                    FieldLine_tmp.start_point = Point(x2,y2);
                    FieldLine_tmp.end_point = Point(x1,y1);
                }
            }
            field_list.push_back(FieldLine_tmp);
        }
    }
    // for(int i = 0; i < field_point.size(); i++ )
    // {
    //     vector<Point> field_tmp = field_point[i];
    //     for(int j = 0; j < field_tmp.size(); j++)
    //     {
    //         function_data field_function_data_tmp;
    //         if(j<field_tmp.size()-1)
    //         {
    //             int x1 = field_tmp[j].x;
    //             int y1 = field_tmp[j].y;
    //             int x2 = field_tmp[j+1].x;
    //             int y2 = field_tmp[j+1].y;
    //             if((y1-y2) == 0)
    //             {
    //                 field_function_data_tmp.linetype = LineType::Horizintal;
    //                 field_function_data_tmp.x = 0.0;
    //             }else if((x1-x2) == 0)
    //             {
    //                 field_function_data_tmp.linetype = LineType::Vertical;
    //                 field_function_data_tmp.x = x1;
    //             }
    //             field_function_data_tmp.para_a = (float)(y1-y2)/(x1-x2);
    //             field_function_data_tmp.para_b = (float)(x1*y2-x2*y1)/(x1-x2);
    //             if(y1>y2)
    //             {
    //                 FOV_function_data_tmp[j].start_point = Point(x1,y1);
    //                 FOV_function_data_tmp[j].end_point = Point(x2,y2);
    //             }else{
    //                 FOV_function_data_tmp[j].start_point = Point(x2,y2);
    //                 FOV_function_data_tmp[j].end_point = Point(x1,y1);
    //             }
    //             field_function_list.push_back(field_function_data_tmp);
    //         }else{
    //             int x1 = field_tmp[j].x;
    //             int y1 = field_tmp[j].y;
    //             int x2 = field_tmp[0].x;
    //             int y2 = field_tmp[0].y;
    //             if((y1-y2) == 0)
    //             {
    //                 field_function_data_tmp.linetype = LineType::Horizintal;
    //                 field_function_data_tmp.x = 0.0;
    //             }else if((x1-x2) == 0)
    //             {
    //                 field_function_data_tmp.linetype = LineType::Vertical;
    //                 field_function_data_tmp.x = x1;
    //             }
    //             field_function_data_tmp.para_a = (float)(y1-y2)/(x1-x2);
    //             field_function_data_tmp.para_b = (float)(x1*y2-x2*y1)/(x1-x2);
    //             if(y1>y2)
    //             {
    //                 FOV_function_data_tmp[j].start_point = Point(x1,y1);
    //                 FOV_function_data_tmp[j].end_point = Point(x2,y2);
    //             }else{
    //                 FOV_function_data_tmp[j].start_point = Point(x2,y2);
    //                 FOV_function_data_tmp[j].end_point = Point(x1,y1);
    //             }
    //             field_function_list.push_back(field_function_data_tmp);
    //         } 
    //     }
    // }
    
    
    
    // function_data field_function_data_tmp;
    // field_function_data_tmp.linetype = LineType::Horizintal;
    // field_function_data_tmp.para_a = 0.0;
    // field_function_data_tmp.para_b = 122.0;
    // field_function_data_tmp.x = 0.0;
    // field_function_data_tmp.start_point = Point(102,122);
    // field_function_data_tmp.end_point = Point(998,678);
    // field_function_list.push_back(field_function_data_tmp);

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
    ROS_INFO("DrawFOV");
    Mat tmp_FOV = Soccer_Field.clone();
    // Mat tmp_FOV_LINE = Soccer_Field.clone();
    // Mat Mask = Mat::zeros(Soccer_Field.rows,Soccer_Field.cols, CV_8UC3); 

    for(int i = 0; i < particlepoint.size(); i++)
    {
        FOV_Point_tmp.push_back(Point(particlepoint[i].FOV_Bottom_Right.x,particlepoint[i].FOV_Bottom_Right.y));
        FOV_Point_tmp.push_back(Point(particlepoint[i].FOV_Top_Right.x,particlepoint[i].FOV_Top_Right.y));
        FOV_Point_tmp.push_back(Point(particlepoint[i].FOV_Top_Left.x,particlepoint[i].FOV_Top_Left.y));
        FOV_Point_tmp.push_back(Point(particlepoint[i].FOV_Bottom_Left.x,particlepoint[i].FOV_Bottom_Left.y));

        //const Point* ppt[1] = {&tmp[0]};
        //int npt[] = {4};

        cv::polylines(tmp_FOV, FOV_Point_tmp, true, Scalar(128, 128, 128), 1);//第2個引數可以採用contour或者contours，均可
	    //cv::fillPoly(tmp_FOV, ppt, npt,  1, Scalar(128, 128, 128));//fillPoly函式的第二個引數是二維陣列！！
        circle(tmp_FOV, Point(particlepoint[i].postion.x,particlepoint[i].postion.y), 2, Scalar(255, 255, 0), 4);
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
            if(particlepoint[i].featurepoint[j].x != 0 && particlepoint[i].featurepoint[j].y !=0)
            {
                circle(*iframe, Point(particlepoint[i].featurepoint[j].x,particlepoint[i].featurepoint[j].y), 3, Scalar(255, 0, 0), 1);
            }
        }
    }*/
    //return *iframe;
}

Mat Drawing::DrawParticlePoint()
{
    ROS_INFO("DrawParticlePoint");
    Mat ParticlePoint_tmp = Soccer_Field.clone();
    for(int i = 0; i < particlepoint.size(); i++)
    {
        circle(ParticlePoint_tmp, Point(particlepoint[i].postion.x,particlepoint[i].postion.y), 3, Scalar(255, 0, 0), 1);
    }
    return ParticlePoint_tmp;
}

Mat Drawing::DrawRobotPos()
{
    ROS_INFO("DrawRobotPos");
    Mat oframe = Soccer_Field.clone();

    Robot_FOV_tmp.push_back(Point(Robot_Position.FOV_Bottom_Right.x,Robot_Position.FOV_Bottom_Right.y));
    Robot_FOV_tmp.push_back(Point(Robot_Position.FOV_Top_Right.x,Robot_Position.FOV_Top_Right.y));
    Robot_FOV_tmp.push_back(Point(Robot_Position.FOV_Top_Left.x,Robot_Position.FOV_Top_Left.y));
    Robot_FOV_tmp.push_back(Point(Robot_Position.FOV_Bottom_Left.x,Robot_Position.FOV_Bottom_Left.y));

    cv::polylines(oframe, Robot_FOV_tmp, true, Scalar(128, 128, 128), 1);//第2個引數可以採用contour或者contours，均可

    circle(oframe, Point(Robot_Position.postion.x,Robot_Position.postion.y), 5, Scalar(255, 255, 0), 2);
    for(int i = 0; i < Robot_Position.featurepoint_scan_line.size(); i++)
    {
        for(int j = 0; j < Robot_Position.featurepoint_scan_line[i].feature_point.size(); j++)
        {
            if(Robot_Position.featurepoint_scan_line[i].feature_point[j].x != -1 && Robot_Position.featurepoint_scan_line[i].feature_point[j].y != -1)
            {
                circle(oframe, Point(Robot_Position.featurepoint_scan_line[i].feature_point[j].x,Robot_Position.featurepoint_scan_line[i].feature_point[j].y), 3, Scalar(255, 0, 0), 3);
            }
        }
    }

    Robot_FOV_tmp.clear();
    return oframe;
}
