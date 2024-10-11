#ifndef THREAD_H
#define THREAD_H

#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>

#include "thread"
#include "unistd.h"               //UNIX系统服务的函数原型
#include "chrono"                 //是标准模板库中与时间有关的头文
#include "fstream"                //输入输出到指定文件

using namespace cv;
using namespace std;

/**
 * @brief 线程管理类
 * 负责图像生成、图像处理、串口数据接收
 */
class ThreadControl
{

public:
    ThreadControl();          // 线程管理构造函数，用于线程中变量初始化
    void ImageProduce();      // 短焦摄像头获取图像线程
    void ImageProcess(); // 图像处理线程，用于自瞄，能量机关识别
    void WriteFrame();
    void PredictProcess();

};

/**
 * @brief 图像信息，用于线程之间的图像传输,and num
 */
struct ImageData
{
    Mat img;
};

#endif // THREAD_H
