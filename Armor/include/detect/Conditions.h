#include "DetectArmor.h"

struct Armor_Conditions
{
    //        double angle_gain;
    //        double angle_diff_gain;
    //        double width_gain;
    //        double height_gain;

    double gain_yaw;
    double gain_pit;
    double gain;

    void Dynamic_Conditions();

    double Light_Long_Width_Min = 1.2;  //
    double Light_Long_Width_Max = 10.0 + gain_pit * 2; //

    double Light_Angle = 35.0 + gain * 10; // 10
    double Armor_Angle = 35.0 + gain * 10;

    double Light_Length_Difference = 0.55;
    double Light_Width_Difference = 0.6;

    double Light_Angle_Different = 15.0 + gain * 2;
    float Light_Angle_Different_Level = 8.2 + gain * 2;
    float Light_Angle_Different_Big = 7.2 + gain * 2;

    double Light_Area_Different_Min = 0.6; //
    double Light_Area_Different_Max = 2.1;   //

    double Armor_Length_Width_Min = 1.0 - gain_pit * 0.6;
    double Armor_Length_Width_Max = 3.1 + gain_yaw * 3.5; // 只用于区分大小装甲板

    //        const double Armor_Length_Width_Min_Big  = 3.0 + 3.5;
    double Armor_Length_Width_Max_Big = 5.3 + gain_yaw * 0.5; // 4.5

    double Light_Height_Difference = 1.6 + gain * 0.5;
    double Light_Level_Difference = 1.1 + gain * 0.5;

    double Light_Height_Difference_Big = 1.5 + gain * 0.2;
};

#define SCORES 30

struct Score
{
//**** 得分 ****//
    double img_dist; // 图像中心优先
    double ctr_dist; // 上一帧位置优先

    double angle;    //左右灯条角度差
    double level_angle;

    double height;   //左右灯条高度差

    double area;     //Z左右灯条面积差

    double var_length;
//****对应权重****//
    double img_w = 50;
    double ctr_w = 50;

    double angle_w = 22;
    double level_angle_w = 17;

    double height_w = 17;

    double area_w = 7;

    double var_w = 20;

};