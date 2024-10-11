/***
 * ┌───┐   ┌───┬───┬───┬───┐ ┌───┬───┬───┬───┐ ┌───┬───┬───┬───┐ ┌───┬───┬───┐
 * │Esc│   │ F1│ F2│ F3│ F4│ │ F5│ F6│ F7│ F8│ │ F9│F10│F11│F12│ │P/S│S L│P/B│  ┌┐    ┌┐    ┌┐
 * └───┘   └───┴───┴───┴───┘ └───┴───┴───┴───┘ └───┴───┴───┴───┘ └───┴───┴───┘  └┘    └┘    └┘
 * ┌───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───────┐ ┌───┬───┬───┐ ┌───┬───┬───┬───┐
 * │~ `│! 1│@ 2│# 3│$ 4│% 5│^ 6│& 7│* 8│( 9│) 0│_ -│+ =│ BacSp │ │Ins│Hom│PUp│ │N L│ / │ * │ - │
 * ├───┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─────┤ ├───┼───┼───┤ ├───┼───┼───┼───┤
 * │ Tab │ Q │ W │ E │ R │ T │ Y │ U │ I │ O │ P │{ [│} ]│ | \ │ │Del│End│PDn│ │ 7 │ 8 │ 9 │   │
 * ├─────┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴─────┤ └───┴───┴───┘ ├───┼───┼───┤ + │
 * │ Caps │ A │ S │ D │ F │ G │ H │ J │ K │ L │: ;│" '│ Enter  │               │ 4 │ 5 │ 6 │   │
 * ├──────┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴────────┤     ┌───┐     ├───┼───┼───┼───┤
 * │ Shift  │ Z │ X │ C │ V │ B │ N │ M │< ,│> .│? /│  Shift   │     │ ↑ │     │ 1 │ 2 │ 3 │   │
 * ├─────┬──┴─┬─┴──┬┴───┴───┴───┴───┴───┴──┬┴───┼───┴┬────┬────┤ ┌───┼───┼───┐ ├───┴───┼───┤ E││
 * │ Ctrl│    │Alt │         Space         │ Alt│    │    │Ctrl│ │ ← │ ↓ │ → │ │   0   │ . │←─┘│
 * └─────┴────┴────┴───────────────────────┴────┴────┴────┴────┘ └───┴───┴───┘ └───────┴───┴───┘
 */
#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core/bufferpool.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/ml/ml.hpp>
#include <stdio.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ml.hpp>
#include <iostream>
#include <string>
#include <math.h>

#include "Advance.h"
#include "Base.h"
#include "Conditions.h"

using namespace std;
using namespace cv;
using namespace cv::ml;

/**
 * @brief 装甲板相关
 */
//基础功能的实现
class Detect
{
public:

    Mat src_img;

    Mat roi_scan;

    struct Light
    {
        Point2f points_[4];
        double angle;
        double angle_diff;
        double level_angle;

        double height_diff;

        double length;
        double width;

        double area;
        double area_avg;
        double armor_area;

        bool error = 0;

        double score;

        Point2f armor_center;
    };

    struct num
    {
        vector<RotatedRect> rect;
        Point2f center;
    };

    vector<Light> lights_;
    vector<Light> armors_;
    vector<Point2f> points;

    //    vector<RotatedRect>three_;

    void Image_Processing(Mat &src);

    void Gray_Test(Mat &src_img);

    void Matching(Mat &src_img);

    bool Screening(Mat &src_img);

    int num_detect(Mat &num_roi);

    Mat ROI_NUM(Mat input, Point2f a, Point2f b, Point2f c, Point2f d, bool big_or_small);

private:
    Mat sub_src;
    Mat gray_src;
};

class ROI
{
public:
    ROI();
    int roi_scan;
    vector<Point2f> roi_points;

    Ptr<SVM> svm = SVM::create();
    // load train file
    bool roi_open = 0;

    double left;
    double right;
    double up;
    double down;

    Mat input;
    int times;
    Mat output;
    void get_roi();
};

extern ROI roi_new;

void ArmorDetectTask(Mat &img);
