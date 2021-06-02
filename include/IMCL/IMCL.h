#include "Particle/Particle.h"
#include <time.h>

using namespace std;
using namespace cv;

struct movement_data
{
    float  straight;
    float  drift;
    float  rotational;
    int  moving;
    float  dt;

};

struct observation_data
{
    vector<scan_line> featurepoint_scan_line;
    vector<LineINF> landmark_list;
};


class IMonteCarlo : 
{
    private:

    public:
    IMonteCarlo();
    ~IMonteCarlo();
    //a list of particles
    vector<ParticlePoint> particles;
    Particle parts;
    //noises for pose
    Eigen::Vector3d noises;

    //mesurement noise
    Eigen::Matrix2d Q_;

    //weights for each particle
    vector<double> weights;
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
    void Initialize(unsigned int landmark_size)
    void Prediction(const movement_data& u);
    void Update(const vector<observation_data>& observations);
    void KLD_Sampling();
    int TournamentResample(int excellent_particle_num)
    void resample();
    void NoLookField(const movement_data& u);

};
