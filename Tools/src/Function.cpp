#include "Advance.h"
using namespace std;
using namespace cv;

//功能函数
Point2f Light_Center(Point2f a, Point2f b, Point2f c, Point2f d)
{
    float center_y = (a.y + b.y + c.y + d.y) / 4;
    float center_x = (a.x + b.x + c.x + d.x) / 4;
    Point2f center = Point2f(center_x, center_y);
    return center;
}

//功能函数
double Distance(Point2f m, Point2f n)
{
    double distance;
    distance = static_cast<double>(powf(m.x - n.x, 2) + powf(m.y - n.y, 2));
    distance = static_cast<double>(sqrt(distance));
    return distance;
}

void AIM_TAG(Mat input, Point2f center, Scalar color = Scalar(255, 0, 255))
{
    circle(input, center, 18, green, 5, 8);
    line(input, Point2f(center.x + 80, center.y), Point2f(center.x - 80, center.y), color, 2, 8);
    line(input, Point2f(center.x, center.y - 50), Point2f(center.x, center.y + 50), color, 2, 8);
    return;
}
 
void Link_Line(Mat input, Point2f a, Point2f b, Point2f c, Point2f d, Scalar color)
{
    line(input, a, b, color, 2, 8);
    line(input, b, c, color, 2, 8);
    line(input, c, d, color, 2, 8);
    line(input, d, a, color, 2, 8);
    return;
}

double Square(double num)
{
    return num * num;
}