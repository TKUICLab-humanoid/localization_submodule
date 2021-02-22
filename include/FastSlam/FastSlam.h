#include <LocalizationBase/LocalizationBase.h>

class FastSlam : public LocalizationBase
{
    public:
        FastSlam();
        ~FastSlam();
        Eigen::Matrix2d Q_;
        int N;
        void Initialize(ParticlePoint& p,unsigned int landmark_size, int _N);
        void Measurement_model(const ParticlePoint& p, int LandMarkID, Eigen::Vector2d& h, Eigen::MatrixXd& H) ;

};