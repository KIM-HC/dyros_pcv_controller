#include <Eigen/Dense>

#define N_CASTERS 4

typedef Eigen::Matrix<double,2*N_CASTERS,1> VectorQd;
typedef Eigen::Matrix<double,2*N_CASTERS,3> MatrixQd;
typedef Eigen::Matrix<double,3,2*N_CASTERS> MatrixQtd;

// For shorthand use in class constructor initialization list.
#define M3I (Eigen::Matrix3d::Identity())
#define M3Z (Eigen::Matrix3d::Zero())
#define V3Z (Eigen::Vector3d::Zero())
