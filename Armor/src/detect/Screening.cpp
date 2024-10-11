#include <DetectArmor.h>
#include <Aim.h>
#include <SerialPort.h>
using namespace std;
using namespace cv;

static Armor_Conditions conditions;
static PNP pnp;

static int _miss(1);
static int _aim(0);

bool big_or_small;

extern Mat src_;
Point2f last_center;

int kalman_times;

bool Detect::Screening(Mat &src_img)
{
    Data_Get dg;

    if (armors_.size() == 0)
    {

        _miss++;
        if (_miss > _MISS_ + 1)
        {
            _miss = _MISS_ + 1;
        }
#ifdef MISS_SWITCH
        if ((_miss > 0 && _miss <= _MISS_) && _aim == 1)
        {

#ifdef PIC
            if (ui_open)
            {
                putText(src_img, "Anti MISS", Point2f(20, 200), FONT_HERSHEY_SIMPLEX, 0.7, pink, 2, 8);
            }
#endif
            vector<Point2f> last_points;

            last_points.push_back(last_points_[0]);
            last_points.push_back(last_points_[1]);
            last_points.push_back(last_points_[2]);
            last_points.push_back(last_points_[3]);

            // dg.get_xy_data(static_cast<int32_t>(x_angle * 10000.0), static_cast<int32_t>(y_angle * 10000.0), 1, 0);
            // dg.get_xy_data(static_cast<int32_t>(0.0), static_cast<int32_t>(0.0), 1, 0);

            if (!last_points.empty())
            {
                pnp.LockArmor(last_points, src_img);
            }
            else
            {
                dg.get_xy_data(static_cast<int32_t>(0.0), static_cast<int32_t>(0.0), 1, 0);
                serial_.send_data(dg);
            }

            vector<Point2f>().swap(last_points);

            // serial_.send_data(dg);
            return true;
        }
        else
        {
#endif
            vector<double>().swap(x_value_save);
            vector<double>().swap(y_value_save);
            vector<double>().swap(z_value_save);
            vector<double>().swap(theta_save);
            kalman_times = 0;

            roi_new.roi_open = 0;
            _aim = 0;
            dg.get_xy_data(0, 0, 0, 0);
            serial_.send_data(dg);
            return false;

#ifdef MISS_SWITCH
        }
#endif
    }
    else if (armors_.size() == 1)
    {

#ifdef NUMBER_DETECT
        Mat num_img = ROI_NUM(src_, armors_[0].points_[0], armors_[0].points_[1], armors_[0].points_[2], armors_[0].points_[3], big_or_small);
        int num_value(0);
        if (!num_img.empty())
        {
#ifdef PIC
            imshow("num", num_img);
#endif
            // num_detect(num_img);
            num_value = svm_num_detect(roi_new.svm, num_img);
        }

        if (num_value == 2)
            return 0;

#endif
        points.push_back(armors_[0].points_[0]);
        points.push_back(armors_[0].points_[1]);
        points.push_back(armors_[0].points_[2]);
        points.push_back(armors_[0].points_[3]);

        if (armors_[0].width / armors_[0].length > conditions.Armor_Length_Width_Max)
            big_or_small = 1;
        else
            big_or_small = 0;

#ifdef PIC
        if (ui_open)
        {
            putText(src_img, "Screen Pass", Point2f(20, 170), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2, 8);
        }
#endif
    }
    else
    {
        static Score score_;
        Light best_armor;

        double min_score(50000.0);

        for (size_t i = 0; i < armors_.size(); i++)
        {
            armors_[i].score = 0;

#ifdef NUMBER_DETECT
            Mat num_img = ROI_NUM(src_, armors_[i].points_[0], armors_[i].points_[1], armors_[i].points_[2], armors_[i].points_[3], big_or_small);
            int num_value(0);
            if (!num_img.empty())
            {
#ifdef PIC
                imshow("num", num_img);
#endif
                // num_detect(num_img);
                num_value = svm_num_detect(roi_new.svm, num_img);
            }
            if (num_value == 2)
                continue;
#endif

            //排名

            //上一帧装甲板位置优先
            if (ds.aim_open == 1 && _aim == 1)
            {
                score_.ctr_dist = (Distance(armors_[i].armor_center, last_center) / Distance(Point2f(0, 0), Point2f(src_img.cols, src_img.rows))) * score_.ctr_w;
                score_.img_dist = 0;
            }
            else //中心距优先
            {
                score_.img_dist = (Distance(armors_[i].armor_center, Point2f(src_img.cols / 2, src_img.rows / 2)) / Distance(Point2f(0, 0), Point2f(src_img.cols / 2, src_img.rows / 2))) * score_.img_w;
                score_.ctr_dist = 0;
            }
            score_.area = armors_[i].area / armors_[i].area_avg * score_.area_w;

            score_.angle = armors_[i].angle_diff / conditions.Light_Angle_Different * score_.angle_w;

            score_.height = ((armors_[i].height_diff / armors_[i].length) / conditions.Light_Height_Difference) * score_.height_w;

            score_.level_angle = armors_[i].level_angle * score_.level_angle_w;

            double grade = score_.img_dist + score_.ctr_dist + score_.area + score_.angle + score_.height + score_.level_angle;
            // cout << grade << endl;

            // for (size_t j = i; j < armors_.size(); j++)
            // {
            //     double cent_dist = Distance(armors_[i].armor_center, armors_[j].armor_center);

            //     double avg_dist = (armors_[i].width + armors_[j].width) / 2;

            //     if (cent_dist < avg_dist)
            //     {
            //         armors_[i].error = 1;
            //         armors_[j].error = 1;
            //     }
            //     else
            //     {
            //         armors_[i].error = 0;
            //         armors_[j].error = 0;
            //     }

            //     cout << cent_dist << "   " << avg_dist << endl;
            // }

            armors_[i].score = grade;
            if (grade < min_score)
            {
                min_score = grade;
            }
        }

        for (size_t i = 0; i < armors_.size(); i++)
        {
            if (armors_[i].score > SCORES)
                continue;
            if (armors_[i].score > min_score)
                continue;

            // if (armors_[i].error)
            //     continue;

            points.push_back(armors_[i].points_[0]);
            points.push_back(armors_[i].points_[1]);
            points.push_back(armors_[i].points_[2]);
            points.push_back(armors_[i].points_[3]);

            if (armors_[i].width / armors_[i].length > conditions.Armor_Length_Width_Max)
                big_or_small = 1;
            else
                big_or_small = 0;

#ifdef PIC
            if (ui_open)
            {
                putText(src_img, "Screen Pass", Point2f(20, 170), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2, 8);
            }
#endif
        }
    }

    vector<Light>().swap(armors_);

    if (!(points.size() < 4))
    {
        pnp.LockArmor(points, src_img);

        _aim = 1;
        _miss = 0;

#ifdef AREA_DETECT

        roi_new.roi_open = 1;

        //设置 ROI 四点
        if (roi_new.roi_points.size() == 0)
        {
            roi_new.roi_points.push_back(points[0]);
            roi_new.roi_points.push_back(points[1]);
            roi_new.roi_points.push_back(points[2]);
            roi_new.roi_points.push_back(points[3]);
        }
        else
        {
            vector<Point2f>().swap(roi_new.roi_points);

            roi_new.roi_points.push_back(points[0]);
            roi_new.roi_points.push_back(points[1]);
            roi_new.roi_points.push_back(points[2]);
            roi_new.roi_points.push_back(points[3]);
        }

        if (!roi_new.input.empty())
        {
            roi_new.get_roi();
        }
#endif

#ifdef PIC
        Point2f aim_center = Light_Center(points[0], points[1], points[2], points[3]);

        last_center = aim_center;

        if (ui_open)
        {
            if (big_or_small)
            {
                Link_Line(src_img, points[0], points[1], points[2], points[3], red);
                AIM_TAG(src_img, aim_center, red);
                AIM_TAG(src_img, Point(src_img.cols / 2, src_img.rows / 2), green);
            }
            else
            {
                Link_Line(src_img, points[0], points[1], points[2], points[3], pink);
                AIM_TAG(src_img, aim_center, pink);
                AIM_TAG(src_img, Point(src_img.cols / 2, src_img.rows / 2), green);
            }
        }
#endif
        vector<Point2f>().swap(points);
        return 0;
    }
    else
    {
        vector<Point2f>().swap(points);
        return -1;
    }
}