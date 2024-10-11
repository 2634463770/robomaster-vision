/*
 *                                         ,s555SB@@&
 *                                      :9H####@@@@@Xi
 *                                     1@@@@@@@@@@@@@@8
 *                                   ,8@@@@@@@@@B@@@@@@8
 *                                  :B@@@@X3hi8Bs;B@@@@@Ah,
 *             ,8i                  r@@@B:     1S ,M@@@@@@#8;
 *            1AB35.i:               X@@8 .   SGhr ,A@@@@@@@@S
 *            1@h31MX8                18Hhh3i .i3r ,A@@@@@@@@@5
 *            ;@&i,58r5                 rGSS:     :B@@@@@@@@@@A
 *             1#i  . 9i                 hX.  .: .5@@@@@@@@@@@1
 *              sG1,  ,G53s.              9#Xi;hS5 3B@@@@@@@B1
 *               .h8h.,A@@@MXSs,           #@H1:    3ssSSX@1
 *               s ,@@@@@@@@@@@@Xhi,       r#@@X1s9M8    .GA981
 *               ,. rS8H#@@@@@@@@@@#HG51;.  .h31i;9@r    .8@@@@BS;i;
 *                .19AXXXAB@@@@@@@@@@@@@@#MHXG893hrX#XGGXM@@@@@@@@@@MS
 *                s@@MM@@@hsX#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@&,
 *              :GB@#3G@@Brs ,1GM@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@B,
 *            .hM@@@#@@#MX 51  r;iSGAM@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@8
 *          :3B@@@@@@@@@@@&9@h :Gs   .;sSXH@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@:
 *      s&HA#@@@@@@@@@@@@@@M89A;.8S.       ,r3@@@@@@@@@@@@@@@@@@@@@@@@@@@r
 *   ,13B@@@@@@@@@@@@@@@@@@@5 5B3 ;.         ;@@@@@@@@@@@@@@@@@@@@@@@@@@@@
 *  5#@@#&@@@@@@@@@@@@@@@@@@9  .39:          ;@@@@@@@@@@@@@@@@@@@@@@@@@@@;
 *  9@@@X:MM@@@@@@@@@@@@@@@#;    ;31.         H@@@@@@@@@@@@@@@@@@@@@@@@@@:
 *   SH#@B9.rM@@@@@@@@@@@@@B       :.         3@@@@@@@@@@@@@@@@@@@@@@@@@@5
 *     ,:.   9@@@@@@@@@@@#HB5                 .M@@@@@@@@@@@@@@@@@@@@@@@@@B
 *           ,ssirhSM@&1;i19911i,.             s@@@@@@@@@@@@@@@@@@@@@@@@@@S
 *              ,,,rHAri1h1rh&@#353Sh:          8@@@@@@@@@@@@@@@@@@@@@@@@@#:
 *            .A3hH@#5S553&@@#h   i:i9S          #@@@@@@@@@@@@@@@@@@@@@@@@@A.
 *
 *
 *    又看源码，看你妹妹呀！
 */

#include "Aim.h"
#include "Kalman.h"

extern bool big_or_small;

double x_angle(0.0);
double y_angle(0.0);
Point2f last_points_[4];
int dist_(0);

static double up_down(0);
static double left_right(0);
double fast_slow;

static Geometry gt;

static Advance ad;

extern int kalman_times;
extern bool jump_;
extern int count_;

bool PNP::LockArmor(vector<Point2f> points, Mat &src_img)
{
    if (big_or_small)
        solvePnP(world_points_l, points, Intrinsic_Matrix, Distortion, rvec, tvec);
    else
        solvePnP(world_points_s, points, Intrinsic_Matrix, Distortion, rvec, tvec);

#ifdef PIC
    char tvec_z[20], tvec_x[20], tvec_y[20];
    char rot_yaw[20];
    char rot_pit[20];
    char rot_row[20];

    char x_com[20];
    char y_com[20];
    char z_com[20];
#endif

#ifdef PIC
    if (!rvec.empty())
    {
        Mat rotR;
        Rodrigues(rvec, rotR);

        double r11 = rotR.ptr<double>(0)[0];
        // double r12 = rotR.ptr<double>(0)[1];
        // double r13 = rotR.ptr<double>(0)[2];
        double r21 = rotR.ptr<double>(1)[0];
        // double r22 = rotR.ptr<double>(1)[1];
        // double r23 = rotR.ptr<double>(1)[2];
        double r31 = rotR.ptr<double>(2)[0];
        double r32 = rotR.ptr<double>(2)[1];
        double r33 = rotR.ptr<double>(2)[2];

        double theta_x = atan2(r32, r33) / CV_PI * 180;
        double theta_y = atan2(-1 * r31, sqrt(r32 * r32 + r33 * r33)) / CV_PI * 180;
        double theta_z = atan2(r21, r11) / CV_PI * 180;

        if (ui_open)
        {
            sprintf(rot_yaw, "%d", int(theta_x));
            putText(src_img, rot_yaw, Point(20, 50), FONT_HERSHEY_SIMPLEX, 0.7, red, 2, 8);

            sprintf(rot_pit, "%d", int(theta_y));
            putText(src_img, rot_pit, Point(100, 50), FONT_HERSHEY_SIMPLEX, 0.7, red, 2, 8);

            sprintf(rot_row, "%d", int(theta_z));
            putText(src_img, rot_row, Point(180, 50), FONT_HERSHEY_SIMPLEX, 0.7, red, 2, 8);
        }
    }
#endif

    if (!tvec.empty())
    {
#ifdef PIC

        if (ui_open)
        {
            const double *distance_1 = tvec.ptr<double>(0);
            const double *distance_2 = tvec.ptr<double>(1);
            const double *distance_3 = tvec.ptr<double>(2);

            sprintf(tvec_x, "%d", int(distance_1[0]));
            sprintf(tvec_y, "%d", int(distance_2[0]));
            sprintf(tvec_z, "%d", int(distance_3[0]));

            putText(src_img, tvec_x, Point2f(20, 20), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2, 8);
            putText(src_img, tvec_y, Point2f(100, 20), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2, 8);
            putText(src_img, tvec_z, Point2f(points[2].x + 40, points[2].y + 20), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2, 8);
        }
#endif
        double *distance_x = tvec.ptr<double>(0);
        double *distance_y = tvec.ptr<double>(1);
        double *distance_z = tvec.ptr<double>(2);

        const double com_value(0.25);
        //上加下减，左加又减
        switch (keyboard_key)
        {
        case 'w':
            up_down = up_down - com_value;
            break;
        case 's':
            up_down = up_down + com_value;
            break;
        case 'a':
            left_right = left_right - com_value;
            break;
        case 'd':
            left_right = left_right + com_value;
            break;
        case 'q':
            fast_slow = fast_slow - com_value;
            break;
        case 'e':
            fast_slow = fast_slow + com_value;
            break;
        }

#ifdef PIC

        if (ui_open)
        {

            sprintf(x_com, "%.2f", double(left_right + YAW_CORRECT));
            sprintf(y_com, "%.2f", double(up_down + PIT_CORRECT));
            sprintf(z_com, "%.2f", double(fast_slow + BULLET_CORRECT));

            putText(src_img, x_com, Point2f(20, 80), FONT_HERSHEY_SIMPLEX, 0.7, yellow, 2, 8);
            putText(src_img, y_com, Point2f(100, 80), FONT_HERSHEY_SIMPLEX, 0.7, yellow, 2, 8);
            putText(src_img, z_com, Point2f(180, 80), FONT_HERSHEY_SIMPLEX, 0.7, yellow, 2, 8);
        }

#endif

        double x = distance_x[0];
        double y = distance_y[0];
        double z = distance_z[0];

        double x_atanvalue(0.0);
        double y_atanvalue(0.0);

        double pnp_yaw = atan(x / z) * 180.0 / PI;
        double pnp_pit = atan(y / z) * 180.0 / PI;

        this->pnp_yaw = pnp_yaw;

#ifdef SENTRY
        //哨兵视觉识别限位
        if (ds.car_id == 7 && ds.camera_pit_angle - pnp_pit > DETECT_LIMIT)
        {
            Data_Get dg;
            dg.get_xy_data(static_cast<int32_t>(0.0), static_cast<int32_t>(0.0), 0, static_cast<uint8_t>(0));
            serial_.send_data(dg);
            return 0;
        }
#endif

        static int dist(1);

#ifdef SENTRY
        if (ds.car_id == 7)
            dist = Firing_Frequency(z);
#endif

#ifdef COMPENSATION
        //大小装甲板PIT轴补偿
        static double pit_corr(0);

        if (big_or_small)
            pit_corr = PIT_CORRECT_BIG;
        else
            pit_corr = PIT_CORRECT;
#ifndef SENTRY

        //数据储存(预瞄)
        x_value_save.push_back(ds.camera_yaw_angle);
        y_value_save.push_back(ds.camera_pit_angle);
        z_value_save.push_back(z);
        theta_save.push_back(pnp_yaw);

        if (x_value_save.size() == SIZE_ * 2 + 1)
        {
            x_value_save.erase(x_value_save.begin());

            last_x_1 = x_value_save.front();
            last_x_2 = x_value_save[SIZE_];
        }
        if (y_value_save.size() == SIZE_ * 2 + 1)
        {
            y_value_save.erase(y_value_save.begin());

            last_y_1 = y_value_save.front();
            last_y_2 = y_value_save[SIZE_];
        }
        if (theta_save.size() == SIZE_ * 2 + 1)
        {
            theta_save.erase(theta_save.begin());

            last_theta_1 = theta_save.front();
            last_theta_2 = theta_save[SIZE_];
        }
        if (z_value_save.size() == GEO_SIZE * 2 + 1)
        {
            z_value_save.erase(z_value_save.begin());

            last_z_1 = z_value_save.front();
            last_z_2 = z_value_save[SIZE_];
        }

        //反陀螺测试
        int anti(0);
        Point2f aim_center = Light_Center(points[0], points[1], points[2], points[3]);
        anti = ad.Anti_Top(aim_center, x, pnp_yaw, z, y);

        if (jump_)
        {
            dist = 0;

            kf[0].Xt_hat.setZero();
            kf[0].Xt_pre.setZero();

            count_++;
        }
        if (count_ > 10 /*&& ds.aim_open*/)
        {
            jump_ = 0;
        }
        if (y_value_save.size() == SIZE_ * 2 && x_value_save.size() == SIZE_ * 2 && z >= 1600)
        {

#ifdef SENTRY
            double yaw_k = ds.camera_yaw_angle - pnp_yaw - gt.Geometry_Solution(z) * ds.plus * GEOMETRY_Q_GAIN;
#else
            double yaw_k = ds.camera_yaw_angle - pnp_yaw;
#endif
            double at_k_yaw = Yaw_Predict(0);
            double v_k_yaw = Yaw_Predict(1);

            // double pit_k = Vertical_Aim_Compensation(z, y);
            // double z_k = z;
            // double v_k_z = Z_Predict(z, 1);

            kf[0].Estimation(at_k_yaw, v_k_yaw, yaw_k);
            // kf[1].Estimation(Y_Predict(0), Y_Predict(1), pit_k);
            // kf[2].Estimation(0, v_k_z, z_k);

            kf[0].End();
            // kf[1].End();
            // kf[2].End();

            int shoot_count_down = 10;
            if (ds.car_id == 7)
            {
                shoot_count_down == 50;
            }

            if (kalman_times >= shoot_count_down && !jump_ && anti <= 1)
            {
                kalman_yaw_value = (ds.camera_yaw_angle - pnp_yaw) - kf[0].Xt_hat[0];
                // kalman_pitch_value = (ds.camera_pit_angle - pnp_pit) - kf[1].Xt_hat[0];
                // z = kf[2].Xt_hat[0];
            }
            else
            {
                cout <<"kalman close" << endl;
                dist = 1;

                shoot_count_down = 0;

                kalman_yaw_value = 0.0;
                // kalman_pitch_value = Vertical_Aim_Compensation(z, y);
                kalman_times++;
            }

            if (anti == 3)
            {
#ifdef PIC
                if (ui_open)
                {
                    putText(src_img, "Aim Top Mode", Point2f(400, 460), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2, 8);
                }
#endif

                if (ds.car_id != 7)
                {
                    x_atanvalue = pnp_yaw/* - kalman_yaw_value */+ left_right + YAW_CORRECT;
                    x_atanvalue = ad.Aim_Top(z, y, x_atanvalue, dist);
                }
                else
                {
                    x_atanvalue = pnp_yaw + left_right + YAW_CORRECT;
                }
            }
            else
            {
#ifdef PIC
                if (ui_open)
                {
                    putText(src_img, "All Compensation Deploy Success!", Point2f(250, 460), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2, 8);
                }
#endif
                x_atanvalue = pnp_yaw - kalman_yaw_value + left_right + YAW_CORRECT;
            }

            y_atanvalue = Vertical_Aim_Compensation(z, y) + up_down + pit_corr;
        }
        else
        {
#endif
            x_atanvalue = pnp_yaw + left_right + YAW_CORRECT;
            y_atanvalue = Vertical_Aim_Compensation(z, y) + up_down + pit_corr;

        }
#else

        x_atanvalue = atan(x / z) * 180.0 / PI + left_right + YAW_CORRECT;
        y_atanvalue = atan(y / z) * 180.0 / PI + up_down + PIT_CORRECT;
#endif

        Data_Get dg;
        dg.get_xy_data(static_cast<int32_t>(x_atanvalue * 10000.0), static_cast<int32_t>(y_atanvalue * 10000.0), 1, static_cast<uint8_t>(dist));
        serial_.send_data(dg);
        cout<< "x_atanvalue "<<x_atanvalue<<"  "<<y_atanvalue<<endl;
        x_angle = x_atanvalue;
        y_angle = y_atanvalue;
        dist_ = dist;

        last_points_[0] = points[0];
        last_points_[1] = points[1];
        last_points_[2] = points[2];
        last_points_[3] = points[3];
    }
    return 1;
}
