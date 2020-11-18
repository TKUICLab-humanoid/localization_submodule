#include <ParticleFilter/ParticleFilter.h>

#define FILED_LENGTH    900        
#define FILED_WIDTH     600

using namespace std;
using namespace cv;


class Drawing : public ParticleFilter  
{
    private:
        vector<Point> FOV_Point_tmp;
        vector<Point> Robot_FOV_tmp;
    public:

    public:
        Drawing();
        ~Drawing();

        Mat convertTo3Channels(const Mat &binImg);
        Mat DrawFiled();
        Mat DrawFOV();
        Mat DrawRobotPos();
        Mat DrawParticlePoint();
        void DrawFeaturePoint(Mat* iframe);
};