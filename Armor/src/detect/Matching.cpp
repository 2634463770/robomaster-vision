#include "DetectArmor.h"
using namespace std;
using namespace cv;
static Armor_Conditions conditions;

void Detect::Matching(Mat &src_img)
{
    if (lights_.size() > 1)
    {
        Light armor;
        int plot(0);

        for (size_t i = 0; i < lights_.size(); i++)
        {

            for (size_t j = i + 1; j < lights_.size(); j++)
            {

#ifdef DEBUG
                circle(src_img, Point(60, 240), 10, blue, 50);
#endif

                //长度差
                if (lights_[i].length / lights_[j].length < conditions.Light_Length_Difference || lights_[i].length / lights_[j].length > 1 / conditions.Light_Length_Difference)
                    continue;
                    //                if (lights_[i].width / lights_[j].width < conditions.Light_Width_Difference
                    //                        || lights_[i].width / lights_[j].width > 1 / conditions.Light_Width_Difference)continue;

#ifdef DEBUG
                circle(src_img, Point(60, 60), 10, green, 50);
#endif

                //装甲板 左右 角度差
                if (abs(lights_[i].angle - lights_[j].angle) > conditions.Light_Angle_Different)
                    continue; // need change to use average to choose right armor plete

#ifdef DEBUG
                circle(src_img, Point(60, 170), 10, pink, 50);
#endif

                //装甲板 上下 角度差
                float rect_level_angle_1(0.0);
                float rect_level_angle_2(0.0);

                if (static_cast<double>(lights_[i].points_[0].x - lights_[j].points_[0].x) == 0.0)
                {
                    rect_level_angle_1 = 90;
                }
                else
                {
                    rect_level_angle_1 = atan((lights_[i].points_[0].y - lights_[j].points_[0].y) / (lights_[i].points_[0].x - lights_[j].points_[0].x)) * 180.0 / PI;
                }

                if (static_cast<double>(lights_[i].points_[3].x - lights_[j].points_[3].x) == 0.0)
                {
                    rect_level_angle_2 = 90;
                }
                else
                {
                    rect_level_angle_2 = atan((lights_[i].points_[3].y - lights_[j].points_[3].y) / (lights_[i].points_[3].x - lights_[j].points_[3].x)) * 180.0 / PI;
                }

                if (fabs(rect_level_angle_1 - rect_level_angle_2) > conditions.Light_Angle_Different_Level)
                    continue;

                if ((rect_level_angle_1 > 5 && lights_[i].angle > 5) || (rect_level_angle_1 < -5 && lights_[i].angle < -5))
                    continue;
                if ((rect_level_angle_2 > 5 && lights_[j].angle > 5) || (rect_level_angle_2 < -5 && lights_[j].angle < -5))
                    continue;

                //灯条面积差
                // if (lights_[i].area / lights_[j].area > conditions.Light_Area_Different_Max || lights_[i].area / lights_[j].area < conditions.Light_Area_Different_Min)continue;

                //装甲板长宽比
                double param_armor_plate_width(0.0);
                double param_armor_plate_height(0.0);
                if (lights_[i].points_[1].x > lights_[j].points_[0].x)
                {
                    param_armor_plate_width = (Distance(lights_[i].points_[1], lights_[j].points_[0]) + Distance(lights_[i].points_[2], lights_[j].points_[3])) / 2;
                    param_armor_plate_height = (Distance(lights_[i].points_[1], lights_[i].points_[2]) + Distance(lights_[j].points_[0], lights_[j].points_[3])) / 2;
                }
                else if (lights_[i].points_[1].x < lights_[j].points_[0].x)
                {
                    param_armor_plate_width = (Distance(lights_[j].points_[1], lights_[i].points_[0]) + Distance(lights_[j].points_[2], lights_[i].points_[3])) / 2;
                    param_armor_plate_height = (Distance(lights_[i].points_[0], lights_[i].points_[3]) + Distance(lights_[j].points_[1], lights_[j].points_[2])) / 2;
                }

                if (param_armor_plate_width / param_armor_plate_height > conditions.Armor_Length_Width_Max_Big || param_armor_plate_width / param_armor_plate_height < conditions.Armor_Length_Width_Min)
                    continue;

                //灯柱间的高度差
                Point2f rect_1_center = Light_Center(lights_[i].points_[0], lights_[i].points_[1], lights_[i].points_[2], lights_[i].points_[3]);
                Point2f rect_2_center = Light_Center(lights_[j].points_[0], lights_[j].points_[1], lights_[j].points_[2], lights_[j].points_[3]);

                double param_height_y_different_light = static_cast<double>(abs(rect_1_center.y - rect_2_center.y));
                if (param_height_y_different_light / param_armor_plate_height > conditions.Light_Height_Difference)
                    continue;

                //中心点角
                float rect_center_angle(0.0);

                if (static_cast<double>(rect_1_center.x - rect_2_center.x) == 0.0)
                    rect_center_angle = 90.0;
                else
                    rect_center_angle = (rect_1_center.y - rect_2_center.y) / (rect_1_center.x - rect_2_center.x);

                if (static_cast<double>(rect_center_angle) > 0.8 || static_cast<double>(rect_center_angle) < -1 * 0.8)
                    continue;

                if (lights_[i].points_[0].x < lights_[j].points_[1].x)
                {
#ifdef PIC
                    if (ui_open)
                    {
                        line(src_img, lights_[i].points_[0], lights_[j].points_[1], yellow, 2);
                        line(src_img, lights_[j].points_[1], lights_[j].points_[2], yellow, 2);
                        line(src_img, lights_[j].points_[2], lights_[i].points_[3], yellow, 2);
                        line(src_img, lights_[i].points_[3], lights_[i].points_[0], yellow, 2);
                    }
#endif

                    armor.points_[0] = lights_[i].points_[0];
                    armor.points_[1] = lights_[j].points_[1];
                    armor.points_[2] = lights_[j].points_[2];
                    armor.points_[3] = lights_[i].points_[3];

                }
                else
                {
#ifdef PIC
                    if (ui_open)
                    {

                        line(src_img, lights_[j].points_[0], lights_[i].points_[1], orange, 2);
                        line(src_img, lights_[i].points_[1], lights_[i].points_[2], orange, 2);
                        line(src_img, lights_[i].points_[2], lights_[j].points_[3], orange, 2);
                        line(src_img, lights_[j].points_[3], lights_[j].points_[0], orange, 2);
                    }
#endif

                    armor.points_[0] = lights_[j].points_[0];
                    armor.points_[1] = lights_[i].points_[1];
                    armor.points_[2] = lights_[i].points_[2];
                    armor.points_[3] = lights_[j].points_[3];

                }

                armor.armor_center = Light_Center(armor.points_[0], armor.points_[1], armor.points_[2], armor.points_[3]);

                armor.area = abs(lights_[i].area - lights_[j].area);
                armor.area_avg = (lights_[i].area + lights_[j].area) / 2;

                armor.angle_diff = abs(lights_[i].angle - lights_[j].angle);

                armor.height_diff = param_height_y_different_light;

                armor.level_angle = rect_center_angle;

                armor.armor_area = param_armor_plate_width * param_armor_plate_height;

                armor.width = param_armor_plate_width;
                armor.length = param_armor_plate_height;

                armors_.push_back(armor);

#ifdef PIC
                if (ui_open)
                {

                    putText(src_img, "Armor Pass", Point2f(20, 140), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2, 8);
                }
#endif
            }
        }
    }

    vector<Light>().swap(lights_);
}
