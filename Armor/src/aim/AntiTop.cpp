#include "Aim.h"
#include "SerialPort.h"
#include "DetectArmor.h"
using namespace std;
using namespace cv;

bool jump_;
int count_;
extern bool big_or_small;

int Advance::Anti_Top(Point2f center, double pnp_x, double pnp_angle, double pnp_z, double pnp_y)
{
    vec_pic.push_back(pnp_angle);

    // vec_armor_v.push_back()

    if (plot_)
    {
        vec_pixel.push_back(center);
        vec_angle_.push_back(ds.camera_yaw_angle);
        vec_time.push_back(time_once);
        vec_pnp_x.push_back(pnp_x);
        vec_pnp_y.push_back(pnp_y);
        vec_pnp_z.push_back(pnp_z);
        vec_time_now.push_back(time_once);
        vec_pnp_yaw.push_back(pnp_angle);

        t_now = 0.0;

        if (vec_pixel.size() > 2)
        {
            vec_pixel.erase(vec_pixel.begin());
            vec_angle_.erase(vec_angle_.begin());
            vec_pnp_x.erase(vec_pnp_x.begin());
            // vec_pnp_z.erase(vec_pnp_z.begin());
            vec_pnp_yaw.erase(vec_pnp_yaw.begin());
        }

        if (vec_time_now.size() == 0)
        {
            t_now = 0;
        }
        else
        {
            for (size_t i = 0; i < vec_time_now.size(); i++)
            {
                t_now += vec_time_now[i];
            }
        }
    }

    if (vec_pic.size() > 2)
    {
        vec_pic.erase(vec_pic.begin());
    }
    double K_P = (vec_pic[1] - vec_pic[0]) * 1.5;

    // Curve(K_P * 10, green, "K");
    // cout << K_P << endl;

    bool save_(0);

    if (K_P >= ANTI_TOP_THRESHOLD) // 顺时针旋转
    {
        count_top_p_1++;
        count_top_p_2 = 0;
        times_max = x_p;
        x_p = 0;

        plus_minus = 1;
        jump_ = 1;
        count_ = 0;
        save_ = 1;
    }
    else if (K_P <= -1 * ANTI_TOP_THRESHOLD) // 逆时针旋转
    {
        count_top_p_1 = 0;
        count_top_p_2--;
        times_max = x_p;
        x_p = 0;

        plus_minus = -1;
        jump_ = 1;
        count_ = 0;
        save_ = 1;
    }

    this->now_ = ds.camera_yaw_angle - pnp_angle;
    if (plot_ && save_)
    {
        // 1为新出现的装甲板的信息（开始），0为当前位置的装甲板的信息（结束）

        this->z_pre = pnp_z;

        // this->top_s = Distance(vec_pixel[1], vec_pixel[0]);
        // this->top_Os = 2 * sin((vec_angle_[1] - vec_angle_[0]) / 2) * (vec_pnp_z[1] + vec_pnp_z[0]) / 2;
        // this->top_x = vec_pnp_x[1] - vec_pnp_x[0];
        this->top_O = (vec_angle_[1] - vec_pnp_yaw[1]) - (vec_angle_[0] - vec_pnp_yaw[0]);

        this->Ot_pre = vec_angle_[1];
        this->Ot_end = vec_angle_[0];

        this->Xt_pre = vec_pnp_x[1];
        this->Xt_end = vec_pnp_x[0];

        this->start_ = vec_angle_[1] - vec_pnp_yaw[1];
        this->finish_ = vec_angle_[0] - vec_pnp_yaw[0];

        // if (vec_pnp_z.size() > 10 || vec_pnp_y.size() > 10)
        // {
        //     this->z_avt = vec_pnp_z[10];
        //     this->y_avt = vec_pnp_y[10];
        // }
        // else
        // {
        //     this->z_avt = vec_pnp_z.front();
        //     this->y_avt = vec_pnp_y.front();
        // }

        this->T0 = 0.0;

        for (int i = 0; i < vec_time.size(); i++)
        {
            this->T0 += vec_time[i];
            this->z_avt += vec_pnp_z[i];
            this->y_avt += vec_pnp_y[i];
        }
        this->z_avt = this->z_avt / vec_pnp_z.size();
        this->y_avt = this->y_avt / vec_pnp_y.size();

        vector<double>().swap(vec_time);
        vector<double>().swap(vec_pnp_x);
        vector<double>().swap(vec_pnp_z);
        vector<double>().swap(vec_angle_);
        vector<double>().swap(vec_pnp_yaw);
        vector<double>().swap(vec_time_now);
        vector<Point2f>().swap(vec_pixel);
    }

    if (count_top_p_1 != 0 || count_top_p_2 != 0)
    {
        x_p++;
    }
    else
    {
        x_p = 0;
    }

    if (x_p > 150)
    {
        count_top_p_1 = 0;
        count_top_p_2 = 0;

        plot_ = 0;

        T0 = 0.0;

        vector<double>().swap(vec_time);
        vector<double>().swap(vec_pnp_x);
        vector<double>().swap(vec_pnp_z);
        vector<double>().swap(vec_pnp_y);
        vector<double>().swap(vec_angle_);
        vector<double>().swap(vec_pnp_yaw);
        vector<double>().swap(vec_time_now);
        vector<Point2f>().swap(vec_pixel);
    }

    if (count_top_p_1 >= 3 || count_top_p_2 <= -3)
    {
        plot_ = 1;
        return 3;
    }
    else if (count_top_p_1 >= 2 || count_top_p_2 <= -2)
    {
        plot_ = 1;
        return 2;
    }
    else if (count_top_p_1 >= 1 || count_top_p_2 <= -1)
    {
        plot_ = 0;
        return 1;
    }
    else
    {
        plot_ = 0;
        return 0;
    }
}

double Advance::Aim_Top(double &z, double &y, double x_atanvalue, int &dist)
{
    /////////////**反陀螺击打逻辑**////////////
    //**逆时针转top_O为正，顺时针转top_O为负；
    double w = top_O / T0; //平均角速度
    double bullet_time = (z_avt / ds.bullet_velocity) / 1000 + Qt;
    double S_hat = w * bullet_time; //当前转动角度
    //**控制 Qt 可以控制 S_hat 的大小**//

    // z = this->z_avt;
    // y = this->y_avt;

    // double S_hat = atan(S_plus / z) * 180.0f / PI;//将S_plus算出来的距离转换为角度

    double d1 = start_;  //出现的那块装甲板的位置
    double d2 = finish_; //消失的那块装甲板的位置
    // double r1 = d1 - S_hat; //瞄准区间——靠近d1
    // double r2 = d2 + S_hat; //瞄准区间——靠近d2

    double r1(0.0);
    double r2(0.0);

    if (big_or_small)
    {
        r1 = d1 - S_hat - (tan( (68 + Qx) / z_avt)) * plus_minus; //瞄准区间——靠近d1
        r2 = d1 - S_hat + (tan( (68 + Qx) / z_avt)) * plus_minus; //瞄准区间——靠近d2
    }
    else
    {
        r1 = d1 - S_hat - (tan((117 + Qx) / z_avt)) * plus_minus; //瞄准区间——靠近d1
        r2 = d1 - S_hat + (tan((117 + Qx) / z_avt)) * plus_minus; //瞄准区间——靠近d2
    }

    r1 += _OFFSET_;
    r2 -= _OFFSET_;

    if (r1 * plus_minus < d1 * plus_minus)
    {
        cout << "r1 is the max, no add Qx or _OFFSET_ anymore" << endl;
        r1 = d1;
    }
    if (r2 * plus_minus > d2 * plus_minus)
    {
        cout << "r2 is the max, no add Qx or _OFFSET_ anymore" << endl;
        r2 = d2;
    }

    double aim_center(0.0);

    double aim_angle = ds.camera_yaw_angle - x_atanvalue;
    // cout << "S : " << S_hat << "camera_yaw : " << ds.camera_yaw_angle << endl;
    // cout << start_ << "      " << r1 << "     " << r2 << "     " << finish_ << endl;
    // cout << T0 << "   " << t_now << "   " << bullet_time << endl;

    /*
     *           逆时针转
     *       d1 - r1 - r2 - d2
     *           顺时针转
     *       d2 - r2 - r1 - d1
     *
     *        云台角度逆时针增大
     */

    /*
    * 正常情况下：r1 > r2
    * 当 r1 < r2 时，说明敌方小陀螺转速过高难以击打
    * 所以让自瞄瞄准一定点位置进行打击
    * 目前问题：射频控制来提高击打效率待测试

    * 判断目前位置是否在击打区域内
    * 如果在则加上预测值进行击打
    * 如果不在则让云台回到 r1 位置
    */

    // problem
    //射频控制

    //中心云台角（反高速陀螺）
    // aim_center = (start_ + top_O / 2) / 2 + (finish_ - top_O / 2) / 2;
    // if (r1 * plus_minus <= r2 * plus_minus)
    // {
    //     aim_center = r2;
    // }
    // else
    // {
    //     aim_center = r1;
    // }
    // aim_center = r2;
    // if (plus_minus == -1) //逆时针转动
    // {
    // if (bullet_time > T0 || ds.car_id == 1)
    // {
    //     Ot_hat = ds.camera_yaw_angle - aim_center;

    //     if ((now_ > r1) || (now_ < r2))
    //     {
    //         dist = 0;
    //     }
    // }
    // else
    // {

    //     if (T0 - t_now < bullet_time)
    //     {
    //         Ot_hat = ds.camera_yaw_angle - r1;
    //         dist = 0;
    //     }
    //     else
    //     {
    //         if (aim_angle < d2)
    //         {
    //             Ot_hat = ds.camera_yaw_angle - d2;
    //         }
    //         else if (aim_angle > d1)
    //         {
    //             Ot_hat = ds.camera_yaw_angle - d1;
    //         }
    //         else
    //         {
    //             Ot_hat = ds.camera_yaw_angle - aim_angle;
    //         }
    //     }
    // }
    // }
    // else
    // {
    //     if (r1 > r2 || bullet_time > T0 || ds.car_id == 1)
    //     {
    //         Ot_hat = ds.camera_yaw_angle - aim_center;

    //         if ((now_ < start_ && now_ > r2) || ((now_ > finish_ && now_ < r1)))
    //         {
    //             dist = 0;
    //         }
    //     }
    //     else
    //     {

    //         if (T0 - t_now < bullet_time)
    //         {
    //             Ot_hat = ds.camera_yaw_angle - r1;
    //             dist = 0;
    //         }
    //         else
    //         {
    //             if (aim_angle > d2)
    //             {
    //                 Ot_hat = ds.camera_yaw_angle - d2;
    //             }
    //             else if (aim_angle < d1)
    //             {
    //                 Ot_hat = ds.camera_yaw_angle - d1;
    //             }
    //             else
    //             {
    //                 Ot_hat = ds.camera_yaw_angle - aim_angle;
    //             }
    //         }
    //     }
    // }
    
    if (0/*ds.car_id == 1 && bullet_time < T0*/)
    {
        aim_center = r2;
        if (now_ * plus_minus >= r1 * plus_minus && now_ * plus_minus <= r2 * plus_minus)
        {
            dist = 1;
        }
        else
        {
            dist = 0;
        }
        return aim_center - ds.camera_yaw_angle;
    }

    if (bullet_time >= T0 * 0.5)
    {
        aim_center = (start_ + top_O / 2) / 2 + (finish_ - top_O / 2) / 2;
        dist = 1;
    }
    else
    {
        aim_center = ds.camera_yaw_angle - x_atanvalue;
        if (aim_center * plus_minus > d2 * plus_minus)
        {
            aim_center = d2;
        }
        else if (aim_center * plus_minus < d2 * plus_minus)
        {
            aim_center = d1;
        }
        dist = 1;
    }
    

    /*
     * 数据是通过 （云台角度 - PNP角度）进行计算的，使用补偿值自瞄需要将云台角减去并加负号
     */
    // Ot_hat = -1 * (Ot_hat - ds.camera_yaw_angle);

    return -aim_center + ds.camera_yaw_angle;
}
