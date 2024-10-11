#include "Base.h"
using namespace std;
using namespace cv;
using namespace cv::ml;
//***函数定义***//
//功能函数
Point2f Light_Center(Point2f a, Point2f b, Point2f c, Point2f d);

//算两点距离
double Distance(Point2f m, Point2f n);

//计算中心点
Point2f Light_Center(Point2f a, Point2f b, Point2f c, Point2f d);

//连线
void Link_Line(Mat input, Point2f a, Point2f b, Point2f c, Point2f d, Scalar color);

// Set the image to a handsome target
void AIM_TAG(Mat input, Point2f aim_center, Scalar color);

void Rotate(const Mat &srcImage, Mat &dstImage, double angle);

int svm_num_detect(Ptr<SVM> svm, Mat img);

double Square(double num);

void ModeSwitch(Mat &image);

void KalmanSwitch();

void Bullet_Velocity_Control();
