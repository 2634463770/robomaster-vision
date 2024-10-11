#pragma once
#include "Base.h"

class Top
{
public:
    double top_s = 0.0;

    //数据帧
    double T0 = 0.0;
    double t_now = 0.0;

    //云台角度减去PNP角度
    double Ot = 0.0;

    //单PNP角度
    double Xt = 0.0;

    double start_ = 0.0;
    double finish_ = 0.0;
};

class Advance
{
public:
    void Curve(double value, Scalar color_, string window_name);
    void Curve2(double value1, double value2);

    int Anti_Top(Point2f center, double pnp_x, double pnp_angle, double pnp_z, double pnp_y);

    double Aim_Top(double &z, double &y, double x_atanvalue, int &dist);

    vector<Point2f> top_coordinate;

    bool plot_;
    int plot;

public:
    vector<Point2f> vec_pixel;
    vector<double> vec_angle_;
    vector<double> vec_time;
    vector<double> vec_pnp_x;
    vector<double> vec_pnp_y;
    vector<double> vec_pnp_z;
    vector<double> vec_time_now;
    vector<double> vec_armor_v;
    vector<double> vec_pnp_yaw;

    vector<Top> top_;

    double top_s;
    double top_Os;
    double top_x;
    double top_O;

    int plus_minus = 0;

    double T0 = 0.0;
    double t_now = 0.0;

    double Ot_hat = 0.0;
    double Ot_now = 0.0;
    double Ot_pre = 0.0;
    double Ot_end = 0.0;

    double Xt_hat = 0.0;
    double Xt_now = 0.0;
    double Xt_pre = 0.0;
    double Xt_end = 0.0;

    double start_ = 0.0;
    double finish_ = 0.0;
    double now_ = 0.0;

    double pnp_angle = 0.0;
    double armor_v = 0.0;
    double z_pre = 0.0;

    double z_avt = 0.0;
    double y_avt = 0.0;

public:
    double times_max = 1;
    double x_p = 0;
    int count_top_p_1 = 0;
    int count_top_p_2 = 0;
    vector<double> vec_pic;
};