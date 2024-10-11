#include "Aim.h"
using namespace std;
using namespace cv;

PNP::PNP()
{
    //判断大装甲和小装甲板
    world_l.clear();
    world_s.clear();

    world_l.push_back(Point3f(-117.5, -34, 0));
    world_l.push_back(Point3f(117.5, -34, 0));
    world_l.push_back(Point3f(117.5, 34, 0));
    world_l.push_back(Point3f(-117.5, 34, 0));
    Mat(world_l).convertTo(world_points_l, CV_64F);

    // 69  32  无膨胀
    world_s.push_back(Point3f(-68, -38, 0));
    world_s.push_back(Point3f(68, -38, 0));
    world_s.push_back(Point3f(68, 38, 0));
    world_s.push_back(Point3f(-68, 38, 0));
    Mat(world_s).convertTo(world_points_s, CV_64F);

    //设置内置参数
    //短焦
#ifdef SHORT

    Intrinsic_Matrix = Mat::eye(3, 3, CV_64F);
    Intrinsic_Matrix.at<double>(0, 0) = 1313.556856647713;
    Intrinsic_Matrix.at<double>(0, 2) = 302.412469653367;
    Intrinsic_Matrix.at<double>(1, 1) = 1313.924431868701;
    Intrinsic_Matrix.at<double>(1, 2) = 253.392308733315;
    Intrinsic_Matrix.at<double>(2, 2) = 1;

    Distortion = Mat::zeros(5, 1, CV_64F);
    Distortion.at<double>(0, 0) = -0.1903;
    Distortion.at<double>(1, 0) = -0.1527;
    Distortion.at<double>(2, 0) = 0;
    Distortion.at<double>(3, 0) = 0;
    Distortion.at<double>(4, 0) = 0;

#else
    Intrinsic_Matrix = Mat::eye(3, 3, CV_64F);
    Intrinsic_Matrix.at<double>(0, 0) = 2656.8;
    Intrinsic_Matrix.at<double>(1, 1) = 2656.1;
    Intrinsic_Matrix.at<double>(0, 2) = 640.4;
    Intrinsic_Matrix.at<double>(1, 2) = 469.5;
    Intrinsic_Matrix.at<double>(2, 2) = 1;

    Distortion = Mat::zeros(5, 1, CV_64F);
    Distortion.at<double>(0, 0) = -0.1999;
    Distortion.at<double>(1, 0) = 0.2066;
    Distortion.at<double>(2, 0) = 0;
    Distortion.at<double>(3, 0) = 0;
    Distortion.at<double>(4, 0) = 0;
#endif

    rvec = Mat::zeros(3, 1, CV_64F);
    tvec = Mat::zeros(3, 1, CV_64F);
}