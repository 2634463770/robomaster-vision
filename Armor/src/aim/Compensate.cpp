#include "Aim.h"
using namespace std;
using namespace cv;

extern double fast_slow;

double PNP::Vertical_Aim_Compensation(double z_distance, double y_distance)
{
    double theta = ds.camera_pit_angle * PI / 180.0;
    double s = (z_distance * cos(theta) + y_distance * tan(theta)) / 1000;
    double h = (z_distance + y_distance * tan(theta)) * sin(theta) / 1000;
    double v = ds.bullet_velocity + fast_slow + BULLET_CORRECT;
    // double v = 18.0 + up_down;

    //  24.7m/s   -->   -2.0
    //  15.2m/s   -->   -1.0

    h += (y_distance / cos(theta)) / 1000;

    double alpha = (asin((h - GRAVITY * (s * s) / (v * v)) / sqrt(s * s + h * h)) + atan(h / s)) / 2;

    return (alpha - theta) * 180 / PI;
}

//预描
double PNP::X_Predict(int num)
{
    double w1 = (last_x_1 - last_x_2) / static_cast<double>(time_all);
    double w2 = (ds.camera_yaw_angle - last_x_1) / static_cast<double>(time_all);
    double p1 = (w2 - w1) / static_cast<double>(time_all);

    if (num == 1)
    {
        return w2;
    }
    else
    {
        return p1;
    }
}

double PNP::Yaw_Predict(int num)
{
    double w1 = ((last_x_1 - last_theta_1) - (last_x_2 - last_theta_2)) / static_cast<double>(time_all);
    double w2 = ((ds.camera_yaw_angle - pnp_yaw) - (last_x_1 - last_theta_1)) / static_cast<double>(time_all);
    double p1 = (w2 - w1) / static_cast<double>(time_all);

    if (num == 1)
    {
        return w2;
    }
    else
    {
        return p1;
    }
}


double PNP::X_Predict_pnp(double x, int num)
{
    double v1 = (last_theta_2 - last_theta_1) / static_cast<double>(time_all);
    double v2 = (x - last_theta_2) / static_cast<double>(time_all);
    double a1 = (v2 - v1) / static_cast<double>(time_all);

    if (num == 1)
    {
        return v2;
    }
    else
    {
        return a1;
    }
}

double PNP::Y_Predict(int num)
{
    double w1 = (last_y_1 - last_y_2) / static_cast<double>(time_all);
    double w2 = (ds.camera_pit_angle - last_y_1) / static_cast<double>(time_all);
    double p1 = (w2 - w1) / static_cast<double>(time_all);

    if (num == 1)
    {
        return w2;
    }
    else
    {
        return p1;
    }
}

double PNP::Z_Predict(double z, int num)
{
    double v1 = (last_z_2 - last_z_1) / static_cast<double>(time_all);
    double v2 = (z - last_z_2) / static_cast<double>(time_all);
    double a1 = (v2 - v1) / static_cast<double>(time_all);

    if (num == 1)
    {
        return v2;
    }
    else
    {
        return a1;
    }
}

int PNP::Firing_Frequency(double z)
{
    if (z <= FREQUENCY)
    {
        return 1;
    }
    else if (z > FREQUENCY + 0000 && z <= FREQUENCY + 1000)
    {
        return 2; // 2
    }
    else if (z > FREQUENCY + 2000 && z <= FREQUENCY + 3000)
    {
        return 6; // 6
    }
    else if (z > FREQUENCY + 3000 && z <= FREQUENCY + 4000)
    {
        return 12; // 8
    }
    else if (z > FREQUENCY + 4000 && z <= FREQUENCY + 5000)
    {
        return 20; // 16
    }
    else
    {
        return 0;
    }
}