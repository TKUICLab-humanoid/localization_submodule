#include <LocalizationBase/LocalizationBase.h>
#include <time.h>

using namespace std;
using namespace cv;

class ParticleFilter : public LocalizationBase  
{
    private:
        int rand_angle_init;
        int particlepoint_num;
        int excellent_particle_num;

        bool find_best_flag;
        bool localization_flag = true;
    public:
        vector<ParticlePoint> particlepoint;

        ParticlePoint Robot_Position;

        bool Step_flag = false;

        int sendbodyauto_x;
        int sendbodyauto_y;
        int continuous_x;
        int continuous_y;
        int step_count;
    public:
        ParticleFilter();
        ~ParticleFilter();

        void ParticlePointinit();
        void CalcFOVArea(int focus, int top, int bottom, int top_width, int bottom_width, float horizontal_head_angle);
        //bool CheckPointArea(Mat* iframe, int x, int y);
        bool CheckPointArea(ParticlePoint *tmp, int x, int y);
        void FindFeaturePoint();
        void FindBestParticle(scan_line *feature_point_observation_data);
        void FindRobotPosition();
        void AverageRobotPos();
        void CalcNewParticle();

        void CalcFOVArea_averagepos(int focus, int top, int bottom, int top_width, int bottom_width, float horizontal_head_angle);

        void NoLookFiled();

        bool GetFindBestFlag(){return find_best_flag;}
        bool GetLocalizationFlag(){return localization_flag;}
};
