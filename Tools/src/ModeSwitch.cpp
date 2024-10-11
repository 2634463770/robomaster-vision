#include "Function.h"
#include "DetectArmor.h"
#include "Aim.h"
#include "Advance.h"
#include "Kalman.h"
#include "Buff_Task.h"
#include "Camera.h"
using namespace std;
using namespace cv;

Kalman kf[3];
Buff_Class buff;

void ModeSwitch(Mat &image)
{
    // if (ds.DetectionMode == 0)
    if(0)
    {
        // cout<<ds.DetectionMode<<endl;
#ifdef COMPENSATION
        KalmanSwitch();
#endif

        CameraMode = 0; //关闭硬触发
        GALAXY_EXPOSURE_TIME = 1500;
        buff.clear_data();//切换装甲板前需要执行buff.clear_data();(只执行一次的)
        ArmorDetectTask(image);
    }
    // else if (ds.car_id >= 3 && ds.car_id <= 5 && ds.DetectionMode >= 1)
    else if(1)
    {
#ifdef INFANTRY

        extern  atomic<int> shootmode;
        shootmode = ds.DetectionMode-1;
        CameraMode = 1;
        // shootmode = 1;
        buff.Buff_Task(image);
#endif
        cout << "Aim buff!!!!!" << shootmode<<endl;
    }
    else
    {
        // LongRangeAttack(image);
    }
}

void KalmanSwitch()
{
    double kalman_plus(0.0);

    if (ds.car_id == 1)
    {
        kalman_plus = KALMAN_GAIN_HERO;
    }
    else
    {
        kalman_plus = Kalman_yaw_V;
    }

    // yaw
    kf[0].Q << Kalman_yaw_Q, 0, 0, kalman_plus; // hold speed
    kf[0].R << Kalman_yaw_R;                    // predict value

    // pitch
    kf[1].Q << Kalman_pit_Q, 0, 0, Kalman_pit_V; // hold speed
    kf[1].R << Kalman_pit_R;                     // predict value

    // z
    kf[2].Q << Kalman_z_Q, 0, 0, Kalman_z_V; // hold speed
    kf[2].R << Kalman_z_R;                   // predict value

    kf[0].time_init(time_once);
    kf[1].time_init(time_once);
    kf[2].time_init(time_once);
}