#include <eigen3/Eigen/Dense>
#include <vector>
#include <math.h>
#include "Base.h"

class Kalman
{
public:

    Kalman();

    void Kalman_init();

    void time_init(double dt);

    void Estimation(double at_x, double v, double yaw_solution);

    void Update();

    void End();

    Eigen::Vector2d Xt_hat, Xt_now, Xt_pre;
    Eigen::Matrix2d Pt_hat, Pt_now, Pt_pre;

//private:
    // Matrices for computation
    Eigen::Matrix<double, 1, 2> H;
    Eigen::Matrix2d Fk;
    Eigen::Vector2d Kt;
    Eigen::Vector2d Bk;
    Eigen::Vector2d Zt;

    Eigen::Matrix<double, 2, 2> Q;
    Eigen::Matrix<double, 1, 1> R;

    // Discrete time step
    double dt;

    // Is the filter initialized?
    bool initialized;

    // n-size identity
    Eigen::Matrix2d I;

    vector<double>angle;
    vector<double>speed;

};

extern Kalman kf[3];

