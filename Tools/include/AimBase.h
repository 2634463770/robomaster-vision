#include <iostream>

//阈值参数    // 110 200
#define CHANNAL_THREAD_BLUE 90
#define GRAY_THREAD_BLUE 50
#define CHANNAL_THREAD_RED 90
#define GRAY_THREAD_RED 40

#define GREEN_CHANNAL_THREAD 30
#define GREEN_GRAY_THREAD 180

//哨兵识别限位
#define DETECT_LIMIT 0

//自瞄硬性补偿
#define YAW_CORRECT -2.0
#define PIT_CORRECT 4.25
#define PIT_CORRECT_BIG 4.5
#define BULLET_CORRECT 4.5

//移动误差的增益值
#define GEOMETRY_Q_GAIN 2.5
#define GEOMETRY_V_GAIN 4.0

//移动误差的计算量
#define GEO_SIZE 5

//子弹射击频率
#define FREQUENCY 5000 //单位mm，以该距离为标志，以下距离全力开火，以上每1米降一定的射频

//进入反陀螺的阈值
#define ANTI_TOP_THRESHOLD 4

//Kalman超参数
// #define KALMAN_PLUS 4.6
#define KALMAN_GAIN_16 2400
#define KALMAN_GAIN_18 2870
#define KALMAN_GAIN_30 3400
#define KALMAN_GAIN_HERO 1400

//卡尔曼预瞄增益量//11   1800   2.8505    //3.1  400  0.8205
#define Kalman_yaw_Q 4.0
#define Kalman_yaw_V 3800
#define Kalman_yaw_R 3.4505

#define Kalman_pit_Q 1.1
#define Kalman_pit_V 180
#define Kalman_pit_R 0.03

#define Kalman_z_Q 0.03
#define Kalman_z_V 0.02
#define Kalman_z_R 1.0

//*************************
//* Q为预瞄为预瞄与实际贴合程度    太大预瞄会超前 收敛变慢，太小预瞄会滞后,
//* V为预瞄收敛程度              太大收敛快，但预瞄量会降低，太小收敛慢，但预瞄量会增大
//* R为预瞄收敛响应速度           太大响应会变慢，太小响应快但会有其他误差影响
//* Q与V会互相影响，参数在调整时需注意控制变量
//*
//* Q, V, R成比例关系
//* Q， V与收敛程度有关
//* Q， V， R会共同对预瞄量产生影响
//* R与响应速度有关
//*************************

//反陀螺
#define TIME_ERROR 0.00
#define Qx -1       //Qx越大，击打区间越窄
#define Qt 0.00
#define _OFFSET_ 0.0  //offset增大击打区间整体往左移，反之
