/*
//
//                       .::::.
//                     .::::::::.
//                    :::::::::::
//                 ..:::::::::::'
//              '::::::::::::'
//                .::::::::::
//           '::::::::::::::..
//                ..::::::::::::.
//              ``::::::::::::::::
//               ::::``:::::::::'        .:::.
//              ::::'   ':::::'       .::::::::.
//            .::::'      ::::     .:::::::'::::.
//           .:::'       :::::  .:::::::::' ':::::.
//          .::'        :::::.:::::::::'      ':::::.
//         .::'         ::::::::::::::'         ``::::.
//     ...:::           ::::::::::::'              ``::.
//    ```` ':.          ':::::::::'                  ::::..
//                       '.:::::'                    ':'````..
//
*/

#include "Aim.h"
#include "SerialPort.h"
#include "DetectArmor.h"

static Armor_Conditions conditions;
using namespace std;
using namespace cv;

Mat src_;
bool ones(1);

void ArmorDetectTask(Mat &img)
{
    img.copyTo(src_);

    Detect armor;

    armor.Image_Processing(img);

    conditions.Dynamic_Conditions();

    img.copyTo(armor.src_img);

    armor.Gray_Test(img);
    armor.Matching(img);
    armor.Screening(img);

#ifdef DEBUG
    imshow("src", armor.src_img);
    imshow("src", img);
#endif
    return;
}

void Detect::Image_Processing(Mat &img)
{
    vector<Mat> splited;
    split(img, splited);

    static int threshold_channal_value(CHANNAL_THREAD_RED);
    static int threshold_gray_value(GRAY_THREAD_RED);

    vector<bool> save_color;
    save_color.push_back(ones);
    if (save_color.size() == 2)
    {
        save_color.erase(save_color.begin());
        if (save_color[0] != save_color[1])
        {
            ones = 1;
        }
    }

    //红色阈值偏低大概20
    if (ds.car_color == 1) //蓝7  红107
    {
        
        if (ones)
        {
            threshold_channal_value = CHANNAL_THREAD_RED;
            threshold_gray_value = GRAY_THREAD_RED;

            ones = 0;
        }

        subtract(splited[2], splited[0], sub_src); // 1 == RED
    }
    else
    {
        if (ones)
        {
            threshold_channal_value = CHANNAL_THREAD_BLUE;
            threshold_gray_value = GRAY_THREAD_BLUE;

            ones = 0;
        }

        subtract(splited[0], splited[2], sub_src); // 0 == BLUE
    }

#ifdef PIC
    namedWindow("dst", CV_WINDOW_AUTOSIZE);
    createTrackbar("sub", "dst", &threshold_channal_value, 255);
    createTrackbar("gray", "dst", &threshold_gray_value, 255);
#endif

#ifdef DEBUG
    imshow("1", splited[2]);
    imshow("2", splited[1]);
#endif

#ifdef DEBUG
    imshow("sub_src", sub_src);
#endif

    threshold(sub_src, sub_src, threshold_channal_value, 255, THRESH_BINARY);

    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    dilate(sub_src, sub_src, kernel);

    cvtColor(img, gray_src, CV_BGR2GRAY);

    threshold(gray_src, gray_src, threshold_gray_value, 255, THRESH_BINARY);
    // dilate(gray_src, gray_src, kernel);

#ifdef PIC
    // imshow("sub", sub_src);
    // imshow("gray", gray_src);
#endif
    return;
}

void Detect::Gray_Test(Mat &src_img)
{
    vector<vector<Point>> contours_sub;
    vector<vector<Point>> contours_gray;
    vector<Vec4i> hierachy;

    findContours(sub_src, contours_sub, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    findContours(gray_src, contours_gray, RETR_EXTERNAL, CHAIN_APPROX_NONE);

    Point2f point_[4];

    Light light;

    for (int i = 0; i < contours_gray.size(); i++)
    {

        double area_gray = contourArea(contours_gray[i]);
        if (area_gray < 20.0 || 80000 < area_gray)
            continue;

        for (size_t j = 0; j < contours_sub.size(); j++)
        {

            double area_sub = contourArea(contours_sub[j]);
            if (area_sub < 20.0 || 80000 < area_sub)
                continue;

            if (pointPolygonTest(contours_sub[j], contours_gray[i].front(), false) == -1.0 && pointPolygonTest(contours_sub[j], contours_gray[i].back(), false) == -1.0)
                continue;

            RotatedRect rect = fitEllipse(contours_gray[i]);
            Point2f rect_points[4];
            rect.points(rect_points);

            //点固定   从左上角开始，顺时针，依次是0，1，2，3
            if (rect_points[0].y < rect_points[1].y)
            {
                point_[0] = rect_points[3];
                point_[1] = rect_points[0];
                point_[2] = rect_points[1];
                point_[3] = rect_points[2];
            }
            else
            {
                point_[0] = rect_points[1];
                point_[1] = rect_points[2];
                point_[2] = rect_points[3];
                point_[3] = rect_points[0];
            }

#ifdef DEBUG
            circle(src_img, rect_points[0], 2, red, 3);
            circle(src_img, rect_points[1], 2, blue, 3);
            circle(src_img, rect_points[2], 2, green, 3);
            circle(src_img, rect_points[3], 2, pink, 3);
#endif

#ifdef AREA_DETECT
            //开始Roi筛选
            // cout << roi_new.roi_open << endl;
            if (roi_new.roi_open /* && ds.aim_open == 1*/)
            {
                Point2f l_cent = Light_Center(point_[0], point_[1], point_[2], point_[3]);

#ifdef PIC
                if (ui_open)
                {
                    Point2f one = Point2f(roi_new.left, roi_new.up);
                    Point2f two = Point2f(roi_new.right, roi_new.up);
                    Point2f three = Point2f(roi_new.right, roi_new.down);
                    Point2f four = Point2f(roi_new.left, roi_new.down);

                    circle(src_img, one, 2, red, 3);
                    circle(src_img, two, 2, blue, 3);
                    circle(src_img, three, 2, green, 3);
                    circle(src_img, four, 2, pink, 3);

                    line(src_img, one, two, white, 2);
                    line(src_img, two, three, white, 2);
                    line(src_img, three, four, white, 2);
                    line(src_img, one, four, white, 2);
                }

#endif

                if (l_cent.x < roi_new.left || l_cent.x > roi_new.right)
                    continue;
                if (l_cent.y < roi_new.up || l_cent.y > roi_new.down)
                    continue;
            }
#endif

#ifdef PIC
            if (ui_open)
            {
                for (size_t i = 0; i < 4; i++)
                {
                    line(src_img, rect_points[i], rect_points[(i + 1) % 4], green, 2, 8);
                }
            }
#endif

            //灯条长宽大小比
            double param_light_width_rect_1 = (Distance(point_[0], point_[1]) + Distance(point_[2], point_[3])) / 2;

            double param_light_height_rect_1 = (Distance(point_[0], point_[3]) + Distance(point_[1], point_[2])) / 2;

            // if (param_light_height_rect_1 / param_light_width_rect_1 < conditions.Light_Long_Width_Min || param_light_height_rect_1 / param_light_width_rect_1 > conditions.Light_Long_Width_Max)
            //     continue;
            if (param_light_height_rect_1 / param_light_width_rect_1 > conditions.Light_Long_Width_Max)
                continue;

            // 第一个轮廓的  角度K值
            float rect_1_angle_x = (point_[2].x + point_[3].x) / 2 - (point_[1].x + point_[0].x) / 2;
            float rect_1_angle_y = (point_[2].y + point_[3].y) / 2 - (point_[1].y + point_[0].y) / 2;
            float rect_1_angle = 0.0;

            if (static_cast<double>(rect_1_angle_y) == 0.0)
                rect_1_angle = 90.0;
            else
                rect_1_angle = atan(rect_1_angle_x / rect_1_angle_y) * 180.0 / PI;

            if (static_cast<double>(rect_1_angle) > conditions.Light_Angle ||
                static_cast<double>(rect_1_angle) < -1 * conditions.Light_Angle)
                continue;

            //保存筛选出来的灯条的点
            light.points_[0] = point_[0];
            light.points_[1] = point_[1];
            light.points_[2] = point_[2];
            light.points_[3] = point_[3];

            //灯条的面积
            light.area = static_cast<double>(area_gray);

            //灯条的长宽
            light.width = param_light_width_rect_1;
            light.length = param_light_height_rect_1;

            //灯条的角度
            light.angle = static_cast<double>(rect_1_angle);

            lights_.push_back(light);

#ifdef PIC
            if (ui_open)
            {
                putText(src_img, "Light Pass", Point2f(20, 110), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2, 8);
            }
#endif
        }
    }

    vector<vector<Point>>().swap(contours_sub);
    vector<vector<Point>>().swap(contours_gray);
    vector<Vec4i>().swap(hierachy);
}

static vector<double> angle_yaw_change;
static vector<double> angle_pit_change;

// static double v_yaw_max = 0;
// static double v_pit_max = 0;

//动态条件
void Armor_Conditions::Dynamic_Conditions()
{
    if (!bool_serial)
    {
        gain = 0.0;
    }
    else
    {
        angle_yaw_change.push_back(ds.camera_yaw_angle);
        angle_pit_change.push_back(ds.camera_pit_angle);

        if (angle_yaw_change.size() == ONE_SIZE && angle_pit_change.size() == ONE_SIZE)
        {
            angle_yaw_change.erase(angle_yaw_change.begin());
            angle_pit_change.erase(angle_pit_change.begin());
        }

        double last_yaw = angle_yaw_change.front();
        double last_pit = angle_pit_change.front();

        double v_yaw = (ds.camera_yaw_angle - last_yaw) / time_all;
        double v_pit = (ds.camera_pit_angle - last_pit) / time_all;

        //    if (v_yaw > v_yaw_max) v_yaw_max = v_yaw;
        //    if (v_pit > v_pit_max) v_pit_max = v_pit;

        //    cout << v_yaw_max << "      " << v_pit_max << endl;

        gain_yaw = abs(v_yaw / 1200);
        gain_pit = abs(v_pit / 2750);

        gain = sqrt(gain_yaw * gain_yaw + gain_pit * gain_pit);
    }

    //     = gain_yaw * 10;

    /* * * * *
     * yaw_max = 1025.349; pitch_max = 2550.73 (aim opened)
     * yaw_max = 482.792; pitch_max = 1312.6 (sentry run)
     *
     * yaw_max = 1000; pitch_max = 2500;
     *
     * 测出云台自秒时最快转动速度v_yaw_max,v_pit_max(定值)
     * gain = v_ / v_max
     * gain = gain * 3; // 乘多少依据条件而定
     *
     * min条件用减； max条件用加； （注意条件没有负值）
     * * * * */
}

ROI::ROI()
{
    svm = StatModel::load<SVM>(SVM_PATH);

    if (!svm)
    {
        cout << "Load file failed..." << endl;
    }
}
