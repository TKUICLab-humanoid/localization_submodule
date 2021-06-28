#include <FastSlam/FastSlam.h>
#include <time.h>

using namespace std;
using namespace cv;

enum class Landmarkmode
{
    PARTICLEPIONT = 1,
    ROBOT = 2
};

class ParticleFilter : public FastSlam  
{
    private:
        int rand_angle_init;
        int particlepoint_num;
        int excellent_particle_num;
        
        //////////////////KLD//////////////////
        int min_particlepoint_num;
        float kld_err;                      //varepsilon in papper
        float kld_z;                        //Z(1-sigma) in papper
        vector<Pointdata> non_empty_bin;    //save the non-empty bins
        //////////////////KLD//////////////////
        
        //////////////////WMCL/////////////////
        float total_weight;
        double totalweights;
        float rotation;
        int posx;
        int posy;
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
        vector<Point> FOV_tmp;
        vector<FieldLine_data> field_list;
        vector<ParticlePoint> particlepoint_compare;
        vector<int> factors;
        ParticlePoint Robot_Position;

        bool Step_flag = false;
        //noises for pose
        Eigen::Vector3d noises;

        //mesurement noise
        Eigen::Matrix2d R;
        
        int sendbodyauto_x;
        int sendbodyauto_y;
        int sendbodyauto_theta;
        int continuous_x;
        int continuous_y;
        int continuous_theta;
        int step_count;
        int SigmaIMU;
        bool use_feature_point;
        bool use_lineinformation;
        /////WMCL//////

        int init_robot_pos_x;
        int init_robot_pos_y;
        float init_robot_pos_dir;

        vector<Point> regions;
        void Motion(ParticlePoint &p,float straight = 0., float drift = 0., float rotational = 0., float dt = 0.);
        void Movement(ParticlePoint &p,float straight = 0., float drift = 0., float rotational = 0., float moving = 1., float dt = 0.);
        void GetUpBackUp();
        void GetUpFrontUp();
        /////WMCL//////
        void FindLineInFOV();

    public:
        ParticleFilter();
        ~ParticleFilter();

        void ParticlePointInitialize(unsigned int landmark_size);
        void CalcFOVArea(int focus, int top, int bottom, int top_width, int bottom_width, float horizontal_head_angle);
        bool CheckPointArea(ParticlePoint *tmp, int x, int y, int FOV);
        void FindFeaturePoint();
        void FindBestParticle(scan_line *feature_point_observation_data, LineINF *Line_observation_data);
        void FindRobotPosition(float x_avg, float y_avg);
        int TournamentResample(int excellent_particle_num);
        void StatePredict(const movement_data& u);
        // double ComputeAngLikelihoodDeg(double angle, double base, double std_deviation);
        void LandMarkMode(Landmarkmode mode);
        void GetBestPoseAndLandmark(VectorXd& mu_ );
        void resample();

        //void CalcNewParticle();

        //////////////////KLD//////////////////
        void KLD_Sampling();

        void CalcFOVArea_averagepos(int focus, int top, int bottom, int top_width, int bottom_width, float horizontal_head_angle);

        void NoLookField(const movement_data& u);

        bool GetFindBestFlag(){return find_best_flag;}
        bool GetLocalizationFlag(){return localization_flag;}
        void FindLandMarkInVirtualField(ParticlePoint *particlepoint);

        //bool compare_int(int a, int b);
};
