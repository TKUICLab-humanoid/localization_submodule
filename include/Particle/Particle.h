#include <FastSlam/FastSlam.h>
#include <time.h>

using namespace std;
using namespace cv;

enum class Landmarkmode
{
    PARTICLEPIONT = 1,
    ROBOT = 2
};

class Particle : public FastSlam  
{
    private:
        
    public:
        //a list of particles
        vector<ParticlePoint> particles;
        vector<ParticlePoint> particlepoint_compare;
        ParticlePoint Robot_Position;
        ParticlePoint Particles_Position;

        vector<Point> FOV_tmp;
        vector<FieldLine_data> field_list;
        ParticlePoint particles;
        //mesurement noise
        Eigen::Matrix2d R;
        
        /////WMCL//////
        vector<int> factors;
        double weight;
        double wfactor;
        double rotation;
        int posx;
        int posy;
        int SigmaIMU;
        bool use_feature_point;
        bool use_lineinformation;

        int particlepoint_num;

    public:
        ParticleFilter();
        ~ParticleFilter();

        void Movement(float straight = 0., float drift = 0., float rotational = 0., int moving = 1, float dt = 0.);
        void Motion(float straight = 0., float drift = 0., float rotational = 0., int moving = 1, float dt = 0.);
        void GetUpBackUp();
        void GetUpFrontUp();
        void CalcFOVArea(int focus, int top, int bottom, int top_width, int bottom_width, float horizontal_head_angle);
        bool CheckPointArea(ParticlePoint *tmp, int x, int y);
        void FindFeaturePoint();
        void FindLandMarkInVirtualField(ParticlePoint *particlepoint);
        double calcWeight(scan_line *feature_point_observation_data, LineINF *Line_observation_data);
        void LandMarkMode(Landmarkmode mode);
};
