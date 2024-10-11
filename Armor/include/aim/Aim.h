/****
 ** ░░░░░░░░░░░░░░░░░░░░░░░░▄░░
 ** ░░░░░░░░░▐█░░░░░░░░░░░▄▀▒▌░
 ** ░░░░░░░░▐▀▒█░░░░░░░░▄▀▒▒▒▐
 ** ░░░░░░░▐▄▀▒▒▀▀▀▀▄▄▄▀▒▒▒▒▒▐
 ** ░░░░░▄▄▀▒░▒▒▒▒▒▒▒▒▒█▒▒▄█▒▐
 ** ░░░▄▀▒▒▒░░░▒▒▒░░░▒▒▒▀██▀▒▌
 ** ░░▐▒▒▒▄▄▒▒▒▒░░░▒▒▒▒▒▒▒▀▄▒▒
 ** ░░▌░░▌█▀▒▒▒▒▒▄▀█▄▒▒▒▒▒▒▒█▒▐
 ** ░▐░░░▒▒▒▒▒▒▒▒▌██▀▒▒░░░▒▒▒▀▄
 ** ░▌░▒▄██▄▒▒▒▒▒▒▒▒▒░░░░░░▒▒▒▒
 ** ▀▒▀▐▄█▄█▌▄░▀▒▒░░░░░░░░░░▒▒▒
 ** 单身狗就这样默默地看着你，一句话也不说。
 ****/

#pragma once

#include "SerialPort.h"
#include "DetectArmor.h"
#include "Geometry.h"
#include "Advance.h"
#include <queue>
#include <Base.h>
using namespace std;
using namespace cv;

extern double x_angle, y_angle;
extern Point2f last_points_[4];
extern int dist_;

class PNP
{
public:
    // use solvePnP and add compensation
    PNP();

    bool LockArmor(vector<Point2f> points, Mat &src_img);
    bool LockGreen(vector<Point2f> points, Mat &src_img);

    //世界坐标定义
    vector<Point3f> world_l;
    Mat world_points_l; //大装甲板

    vector<Point3f> world_s;
    Mat world_points_s; //小装甲板

    //相机内置参数， 畸变函数定义
    Mat Intrinsic_Matrix;
    Mat Distortion;

    //平移向量， 旋转矩阵定义
    Mat rvec;
    Mat tvec;

    double X_Predict(int num);
    double Yaw_Predict(int num);

    double X_Predict_pnp(double x, int num);

    //    inline double Y_Predict(double z_differ, double y);
    double Y_Predict(int num);

    double Z_Predict(double z, int num);

    int Firing_Frequency(double z);

    // Calculate the Vertical Compensation
    double Vertical_Aim_Compensation(double z_distance, double y_distance);

    double last_camera_yaw = 0.0;
    double last_z_1 = 0.0;
    double last_z_2 = 0.0;

    double last_x_1 = 0.0;
    double last_x_2 = 0.0;

    double last_y_1 = 0.0;
    double last_y_2 = 0.0;

    double last_theta_1 = 0.0;
    double last_theta_2 = 0.0;

    double pnp_yaw = 0.0;

    double anti_top = 0.0;

    double kalman_yaw_value = 0.0;
    double kalman_pitch_value = 0.0;
    double kalman_plus = 0.0;
};

static vector<double> z_value_save;
static vector<double> x_value_save;
static vector<double> y_value_save;
static vector<double> theta_save;