/***
 *                   /88888888888888888888888888\
 *                   |88888888888888888888888888/
 *                    |~~____~~~~~~~~~"""""""""|
 *                   / \_________/"""""""""""""\
 *                  /  |              \         \
 *                 /   |  88    88     \         \
 *                /    |  88    88      \         \
 *               /    /                  \        |
 *              /     |   ________        \       |
 *              \     |   \______/        /       |
 *   /"\         \     \____________     /        |
 *   | |__________\_        |  |        /        /
 * /""""\           \_------'  '-------/       --
 * \____/,___________\                 -------/
 * ------*            |                    \
 *   ||               |                     \
 *   ||               |                 ^    \
 *   ||               |                | \    \
 *   ||               |                |  \    \
 *   ||               |                |   \    \
 *   \|              /                /"""\/    /
 *      -------------                |    |    /
 *      |\--_                        \____/___/
 *      |   |\-_                       |
 *      |   |   \_                     |
 *      |   |     \                    |
 *      |   |      \_                  |
 *      |   |        ----___           |
 *      |   |               \----------|
 *      /   |                     |     ----------""\
 * /"\--"--_|                     |               |  \
 * |_______/                      \______________/    )
 *                                               \___/
 */

#include "Geometry.h"
using namespace std;
#define IS_SENTRY
vector<double> speed_save;

double Geometry::Geometry_Solution(double y2)
{
    // y3 X

    //哨兵陀螺仪逆时针为正，指左边为0
    //英雄步兵陀螺仪逆时针为正，指前面为0

    //车身移动左正右负
    //往右移为减 ；往左移为加

    // ds.car_speed_x 单位 : m / s
    // dist速度单位：mm / ms
    speed_save.push_back(ds.car_speed_x);

    double a(0);

    if (speed_save.size() > 2)
    {
        speed_save.erase(speed_save.begin());

        a = (speed_save.back() - speed_save.front()) / time_once;
    }

    double dist = (ds.car_speed_x * time_once + (a * Square(time_once)) / 2) * 1000 * GEOMETRY_V_GAIN;

    double Ot1 = ds.camera_yaw_angle;

#ifndef IS_SENTRY
    Ot1 = 90.0;
#endif

RESET:
    if (Ot1 >= 360)
    {
        Ot1 = Ot1 - 360;
        goto RESET;
    }
    else if (Ot1 <= -360)
    {
        Ot1 = Ot1 + 360;
        goto RESET;
    }

    double Ot(0.0);

    if (dist != 0.0)
    {
        double y1 = sqrt(Square(y2) + Square(dist) - 2 * y2 * dist * cos(Ot1 * PI / 180.0));

        double Ot2 = acos((Square(y1) + Square(dist) - Square(y2)) / (2 * y1 * dist)) * 180.0 / PI;

        // cout << "Ot2 : " << Ot2 << endl;
        // cout << "Ot1 : " << Ot1 << endl;
        // cout << "Ot  : " << 180 - (Ot1 + Ot2) << endl;
        Ot = 180 - (Ot1 + Ot2);

        if (Ot > 60)
        {
            Ot = 0.0;
        }
    }

    return Ot;

    // if (ds.car_speed_x == 0.0 || abs((y3 * y3) + (y2 * y2) - (car_distance_x * car_distance_x)) >  abs( 2 * y3 * y2 ))
    // {
    //     Oxy_now = 0.0;
    // }
    // else
    // {
    //     Oxy_now = acos( ((y3 * y3) + (y2 * y2) - (car_distance_x * car_distance_x)) / ( 2 * y3 * y2 ) ) * 180.0 / PI;
    // }
}

vector<double> sentry_addr_save;
double Geometry::Geometry_Solution_New(double pnp_yaw, double armor_at, double armor_vt)
{
    //哨兵陀螺仪逆时针为正，指左边为0
    //英雄步兵陀螺仪逆时针为正，指前面为0

    //车身移动左正右负
    //往右移为减 ；往左移为加

    double Ot;
    sentry_addr_save.push_back(armor_vt);
    double sentry_address;
    double sentry_at;
    if (sentry_addr_save.size() == 2)
    {
        sentry_addr_save.erase(sentry_addr_save.begin());

        sentry_at = (sentry_addr_save[1] - sentry_addr_save[0]) / time_once;

        sentry_address = ds.car_speed_x * time_once;

        
    }
    else
    {
        return 0;
    }
}