/***
 *_______________#########_______________________
 *______________############_____________________
 *______________#############____________________
 *_____________##__###########___________________
 *____________###__######_#####__________________
 *____________###_#######___####_________________
 *___________###__##########_####________________
 *__________####__###########_####_______________
 *_________####___###########__#####_____________
 *________#####___###_########___#####___________
 *_______#####____###__########___######_________
 *______######___###__###########___######_______
 *_____######___####_##############__######______
 *____#######__#####################_#######_____
 *___########_################################____
 *___########_######_#################_#######___
 *___#######__######_######_#########___######___
 *___#######____##__######___######_____######___
 *___#######________######____#####_____#####____
 *____######________#####_____#####_____####_____
 *_____#####________####______#####_____###______
 *______#####______;###________###______#________
 *________##_______####________####______________
 ***/

#ifndef BASE_H
#define BASE_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include "Function.h"
#include "AimBase.h"
#include "Camera.h"
using namespace std;
using namespace cv;
extern int keyboard_key;
//-------------------------------------------------------------------
// 功能状态开关
//-------------------------------------------------------------------
// #define CAMERA                  ///*读取摄像头开关*///
#define VIDEO                      ///*读取视频*///
// #define WRITE                   ///*视频写入开关*///
#define COMPENSATION               ///*补偿开关*///
// #define NotShowImage
// #define WATCHDOG                ///*看门狗开关*///
#define PIC                        ///*debug开关*///
// #define DEBUG                   ///*更细的debug的开关*///
//#define DEBUG_SERIAL             ///*串口debug的开关*///
// #define NUMBER_DETECT           ///*数字识别开关*///
// #define NUMER_DETECT_ANO        ///*另一个ROI开关*///
// #define AREA_DETECT             ///*区域识别*///

#define SHORT                      ///*短焦距开关*///

//**开启每个兵种专有的模式**//
// #define HERO        //英雄
#define INFANTRY     //步兵
// #define SENTRY      //哨兵

//waitkey的值
#define WAITKEY_VALUE 1
//watch dog 容错次数
#define FULL_DOG 800

// 曝光时间
// #define GALAXY_EXPOSURE_TIME 1000 // 在Camera.h，修改的时候直接赋值就行，GALAXY_EXPOSURE_TIME=2000


//装甲板防闪烁的周期
#define _MISS_ 15
//装甲板防闪烁开关
#define MISS_SWITCH


///*识别到后xx 帧后开启卡尔曼*///
#define KALMAN_TIME_OPEN  
///*卡尔曼开关*///         
#define KALMAN_SWITCH 1             


//卡尔曼的计算周期
#define SIZE_ 1
#define ONE 1
#define ONE_SIZE ONE + 1
#define TWO ONE * 2
#define TWO_SIZE TWO + 1
#define BLUE true


//串口
#define SERIAL_VMIN 21
#define SERIAL_VTIME 0
#ifdef INFANTRY
#define READ_SIZE 21
#else
#define READ_SIZE 16
#endif

//-------------------------------------------------------------------
// 视频路经（需要打开视频识别开关）
//-------------------------------------------------------------------
// #define ADD "/home/u0/Videos/race_2.avi"
// #define ADD "/home/u0/Videos/real/solidar.avi"
// #define ADD "/home/arno/image/Buff_Bug1.avi"
// #define ADD "/home/u0/Videos/top/3.mp4"
// #define ADD "/home/arno/Videos/big_armor_2.avi"
// #define ADD "/home/u0/Videos/top/small/1.avi"
// #define ADD "/home/u0/Videos/test.avi"
// #define ADD "/home/u0/Videos/test/test_07.avi"
// #define ADD 0
// #define ADD "/home/u0/Videos/top/0506/6.avi"
// #define ADD "/home/u0/Videos/反陀螺慢动作效果.mp4"
// #define ADD "/home/u0/Videos/race/2021-RMUC-SZGS-去op.mp4"
// #define ADD "/home/u0/Videos/LIGHT12.avi"
#define ADD "/home/arno/Videos/buff/Buff_Bug1.avi"
// #define ADD "/home/arno/Videos/buff/RM2019能量机关视频（位于桥上拍摄）/机器人视角 红色 背景暗.avi"
// #define ADD "/home/arno/Videos/buff/RM2019能量机关视频（位于桥上拍摄）/机器人视角 蓝色 背景暗.avi"


//-------------------------------------------------------------------
// serial port paths 串口路径
//-------------------------------------------------------------------
#define SERIAL_PATH "/dev/ttyUSB0"


//-------------------------------------------------------------------
// 参数调整
//-------------------------------------------------------------------
#define PI 3.1415926
#define GRAVITY 9.8


//-------------------------------------------------------------------
// svm path
//-------------------------------------------------------------------
#define SVM_PATH "../Armor/src/svm/svm.xml"


extern double time_all;
extern double time_once;

extern int watch_dog;

extern bool ui_open;

//-------------------------------------------------------------------
// color
//-------------------------------------------------------------------
//用颜色方便点
static Scalar red    = Scalar(0,     0, 255);
static Scalar blue   = Scalar(255,   0,   0);
static Scalar pink   = Scalar(203, 152, 255);
static Scalar green  = Scalar(0,   255,   0);
static Scalar white  = Scalar(255, 255, 255);
static Scalar orange = Scalar(0,   128, 255);
static Scalar yellow = Scalar(0,   255, 255);

#endif // BASE_H
