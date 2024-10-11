#ifndef __BUFFBASE_H__
#define __BUFFBASE_H__
#include <iostream>
#include <stdlib.h>
#include <string>
#include <math.h>
#include "SerialPort.h"
#include <time.h>
#include <stdio.h>
#include <sys/time.h>
#include <Buff_Task.h>
#include <unistd.h>  //access
#include <sys/stat.h>  //mkdir
#include <sys/types.h> //mkdir
#include "Camera.h"
#include <mutex>
#include "omp.h"
// #include <fftw3.h>
#include <opencv2/opencv.hpp>
#include <numeric>      // std::accumulate
using namespace std;
using namespace cv;
//功能开关
#define DEBUG_MODE //调试模式
#define Pixel_PNP //使用像素PNP，maltab拟合曲线，但更准确
//#define Make_Data
// #define DEBUG_ALL
#ifdef DEBUG_ALL
#define DEBUG_PIC //画图
#define DEBUG_LOG //终端输出信息
#else
#define DEBUG_PIC
// #define DEBUG_LOG
#endif
#define hsv false //是否使用HSV  RGB或HSV
#define BUFF_is_BLUE false //能量机关是否蓝色

//独立悬挂麦轮 0 -17 -0.028 -0.45 0.012
#ifdef Pixel_PNP //PNP补偿在像素上补偿的方法
#define Pitch_Compensation 0.0 //pitch补偿
#define Yaw_Compensation -53.0  //yaw补偿

#else //直接在云台角上补偿
#define Pitch_Compensation -0.028  //pitch补偿
#define Yaw_Compensation -0.45 //yaw补偿
#endif
#define PredictTime_Compensation 0.032 //预测时间补偿 

//图像处理
#define Blue_Value 30 //40
#define Red_Value 30 //13
#define Blue_Gray_Value 60 //20
#define Red_Gray_Value 40 //20
//目标
#define area_ratio 6.00//480 //内外轮廓面积比
#define disappear_times 10 //目标最大消失帧数
// 预测类
#define num_mean 2 //n帧角速度的平均值  (v1+v2)/2
#define Theta_Size 360.0 //预测曲线 theta精度， 2PI/Theta_Size
#define V_Data_MaxSize 400.0 //最大存储角速度的大小
#define min_f 0.29 //最小频率
#define max_f 0.325//0.3  0.31831 //最大频率
#define min_A 0.770 //最小振幅
#define max_A 1.055//0.780  1.045 //最大振幅
#define interval_f 0.005 //频率的间隔
#define interval_A 0.03 //振幅的间隔
#define fs 110.0 //外触发帧率


extern bool ui_open;
#define XML_Path "../Buff/V_Data.xml"
#endif