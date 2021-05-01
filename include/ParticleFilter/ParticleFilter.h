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
        
        //////////////////KLD//////////////////
        int min_particlepoint_num;
        float kld_err;                      //varepsilon in papper
        float kld_z;                        //Z(1-sigma) in papper
        vector<pointdata> non_empty_bin;    //save the non-empty bins
        //////////////////KLD//////////////////
        
        //////////////////Augmented_MCL//////////////////
        float weight_avg;       //the average of the weight of particlepoint
        float weight_slow;      //the long-term weight of particlepoint
        float weight_fast;      //the short-term weight of particlepoint
        float alpha_slow;
        float alpha_fast;
        //////////////////Augmented_MCL//////////////////

        bool find_best_flag;
        bool localization_flag;
    public:
        vector<ParticlePoint> particlepoint;
        vector<ParticlePoint> particlepoint_compare;

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
        bool CheckPointArea(ParticlePoint *tmp, int x, int y);
        void FindFeaturePoint();
        void FindBestParticle(scan_line *feature_point_observation_data);
        void FindRobotPosition(float x_avg, float y_avg);
        int TournamentResample(int excellent_particle_num);
        void StatePredict();
        //void CalcNewParticle();

        //////////////////KLD//////////////////
        void KLD_Sampling();

        void CalcFOVArea_averagepos(int focus, int top, int bottom, int top_width, int bottom_width, float horizontal_head_angle);

        void NoLookFiled();

        bool GetFindBestFlag(){return find_best_flag;}
        bool GetLocalizationFlag(){return localization_flag;}

        //bool compare_int(int a, int b);
};
