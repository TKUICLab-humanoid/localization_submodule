#include "ParticleFilter/ParticleFilter.h"
#include <time.h>

using namespace std;
using namespace cv;

class IMonteCarlo : public ParticleFilter  
{
    private:

    public:
    bool use_feature_point;
    bool use_lineinformation;
    double totalweight;
};
