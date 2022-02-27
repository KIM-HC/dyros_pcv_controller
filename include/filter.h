#ifndef _Filter_h_
#define _Filter_h_

#include <Eigen/Dense>


class Filter
{
public:
    Filter();
    ~Filter();

    // Use this after a filter has been initialized to filter the data
    Eigen::VectorXd Filt(Eigen::VectorXd const &in);

    // General digital filter: z_num/zden
    void Z_NumDen(Eigen::VectorXd const &r0,
                  Eigen::VectorXd const &z_num,
                  Eigen::VectorXd const &z_den,
                  Eigen::VectorXd const &f0 );

    // Filters below are digital equivalents using Tustin's method
    // w/(s+w)
    void LowPass (Eigen::VectorXd const &r0, float sampFreq, float cutOffFreq);
    // (s*w)/(s+w)
    void D_LowPass (Eigen::VectorXd const &r0, float sampFreq, float cutOffFreq);

private:
    int size_;
    int order_;

    Eigen::VectorXd *a_;
    Eigen::VectorXd *b_;

    Eigen::VectorXd **f_;
    Eigen::VectorXd **r_;

    void Allocate( int size, int order );
    void Purge();
};

#endif // _Filter_h_
