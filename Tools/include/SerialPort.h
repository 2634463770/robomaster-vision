#ifndef _SERIAL_PORT_H
#define _SERIAL_PORT_H
#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include "Base.h"
using namespace std;
using namespace cv;

struct Data_Save
{
    bool car_color;
    bool mode_num;
    double camera_yaw_angle;
    double camera_pit_angle;
    double bullet_velocity = 20;
    double car_speed_x; //底盘的运动速度
    double car_speed_y;
    double car_angle; //底盘的角度差
    double plus;
    bool aim_open;
    int car_id;

// #ifdef INFANTRY
    int DetectionMode;
    int PitchCompensation;
    int YawCompensation;
    int PredictionCompensation;
    int Auto_fire;
// #endif

    /********注意****
     * 电控代码运行一次的时间；视觉代码运行一次的时间
     * 电控代码与视觉代码之间的时间差
     * *************/
};

extern Data_Save ds;

extern bool bool_serial;

struct Data_Get
{
    u_char raw_data[13];
    int size;
    void get_xy_data(int32_t x, int32_t y, uint8_t found, uint8_t fire);
};

class SerialPort
{
public:
    SerialPort(const char *filename);

    void restart_serial(void); // 尝试重连的函数
    bool read_data();
    void send_data(const struct Data_Get &data);

    int fd;
    int last_fd;
    bool success_;

private:
    const char *file_name_;
    int buadrate_;
    float last_bullet_speed;
};

static SerialPort serial_(SERIAL_PATH);

#endif