#include "Aim.h"
#include "SerialPort.h"
#include "DetectArmor.h"
using namespace std;
using namespace cv;

Mat Detect::ROI_NUM(Mat input, Point2f a, Point2f b, Point2f c, Point2f d, bool big_or_small)
{
    Mat img;

    input.copyTo(img);

    int gain_w(20);
    int gain_h(60);

    int gain_x(10);
    int gain_y(30);

    int width =  static_cast<int>(abs(a.x - c.x)) - gain_w;
    int height = static_cast<int>(abs(a.y - c.y)) + gain_h;

    int x = static_cast<int>(a.x + gain_x);
    int y = static_cast<int>(a.y - gain_y);

    if (width >= input.cols - a.x)
        width = input.cols - a.x - 1;
    else if (width <= 0)
        width = 1;

    if (height >= input.rows - a.y)
        height = input.rows - a.y - 1;
    else if (height <= 0)
        height = 1;

    if (x <= 0)
        x = 1;
    if (y <= 0)
        y = 1;

    double angle = atan((b.y - a.y) / (b.x - a.x)) * 180.0 / PI;

    Mat roi_ = img(Rect(x, y, width, height));

    Rotate(roi_, roi_, angle);

    if (big_or_small)
    {
        resize(roi_, roi_, Size(204, 128));
    }
    else
    {
        resize(roi_, roi_, Size(116, 128));
    }

    return roi_;
}

void ROI::get_roi()
{
    double up;
    double down;
    double left;
    double right;
    // Point2f center = Light_Center(roi_points[0], roi_points[1], roi_points[2], roi_points[3]);

    if (roi_points[0].y > roi_points[1].y)
        up = roi_points[1].y;
    else
        up = roi_points[0].y;

    if (roi_points[0].x > roi_points[3].x)
        left = roi_points[3].x;
    else
        left = roi_points[0].x;

    if (roi_points[1].x > roi_points[2].x)
        right = roi_points[1].x;
    else
        right = roi_points[2].x;

    if (roi_points[2].y > roi_points[3].y)
        down = roi_points[2].y;
    else
        down = roi_points[3].y;

    double roi_up = up - ( down - up ) * 1.5;
    double roi_left = left - (( down - up ) * ( down - up ) * 16) / ( right - left );
    double roi_right = right + (( down - up ) * ( down - up ) * 16) / ( right - left );
    double roi_down = down + ( down - up ) * 1.5;

    if (roi_up < 0)
        roi_up = 0;
    if (roi_left < 0)
        roi_left = 0;
    if (roi_right > input.cols)
        roi_right = input.cols;
    if (roi_down > input.rows)
        roi_down = input.rows;

    if (input.empty())
    {
        cout << "empty" << endl;

        this->roi_open = 0;
    }
    else
    {
        output = input(Rect(roi_left + 1, roi_up + 1, roi_right - roi_left - 1, roi_down - roi_up - 1));

        this->left = roi_left + 1;
        this->right = roi_left + 1 + roi_right - roi_left - 1;
        this->up = roi_up + 1;
        this->down = roi_up + 1 + roi_down - roi_up - 1;

        // if (!output.empty())
        // {
        //     imshow("roi", output);
        // }

    }
}

double i(0);

vector<Point2f> draw_point;

void Advance::Curve(double value, Scalar color_, string window_name)
{
    static Mat img;
    img.create(720, 1200, CV_8UC3);

    value = value + 360;

    Point2f draw = Point2f(i, value);

    Point2f cout_draw = Point2f(i, value - 360);

    draw_point.push_back(draw);

    if (draw_point.size() > 2)
    {
        draw_point.erase(draw_point.begin());

        line(img, draw_point[0], draw_point[1], color_, 2);
    }

    // circle(img, draw, 1, Scalar(0, 0, 255), 2, 8);

    line(img, Point(0, 360), Point(1200, 360), Scalar(255, 255, 0), 1);

    top_coordinate.push_back(draw);

    if (top_coordinate.size() > 20)
    {
        top_coordinate.erase(top_coordinate.begin());
    }
    imshow(window_name, img);
    i += 2;

    if (i > 1160 || draw_point[0].x > draw_point[1].x)
    {
        i = 0;
        img = Mat::zeros(360, 1200, CV_8UC3);
    }
}


double j(0);

vector<Point2f> draw_point2;
vector<Point2f> draw_point3;

void Advance::Curve2(double value1, double value2)
{
    static Mat img;
    img.create(720, 1200, CV_8UC3);

    value1 = value1 + 360;
    value2 = value2 + 360;

    Point2f draw1 = Point2f(j, value1);
    Point2f draw2 = Point2f(j, value2);

    Point2f cout_draw1 = Point2f(j, value1 - 360);
    Point2f cout_draw2 = Point2f(j, value2 - 360);

    draw_point2.push_back(draw1);
    draw_point3.push_back(draw2);

    if (draw_point2.size() > 2 || draw_point3.size() > 2)
    {
        draw_point2.erase(draw_point2.begin());
        draw_point3.erase(draw_point3.begin());

        line(img, draw_point2[0], draw_point2[1], green, 2);
        line(img, draw_point3[0], draw_point3[1], yellow, 2);

    }

    // circle(img, draw, 1, Scalar(0, 0, 255), 2, 8);

    line(img, Point(0, 360), Point(1200, 360), Scalar(255, 255, 0), 1);


    imshow("Curve2", img);
    j += 4;

    if (j > 1160 || draw_point2[0].x > draw_point2[1].x)
    {
        j = 0;
        img = Mat::zeros(720, 1200, CV_8UC3);
    }
}

void Rotate(const Mat &srcImage, Mat &dstImage, double angle)
{
    Point2f center(srcImage.cols / 2, srcImage.rows / 2);                  //中心
    Mat M = getRotationMatrix2D(center, angle, 1);                         //计算旋转的仿射变换矩阵
    warpAffine(srcImage, dstImage, M, Size(srcImage.cols, srcImage.rows)); //仿射变换
    circle(dstImage, center, 2, Scalar(255, 0, 0));
}
