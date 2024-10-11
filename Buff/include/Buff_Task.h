#pragma once
#ifndef BUFF_DETECT_H
#define BUFF_DETECT_H
#include "BuffBase.h"

/*     (w)
 *  0--------1              90度
 *  |        | (h)        /     \
 *  3--------2       (+-)180度   0度
 *     |  |               \     /
 *     |  |                -90度
 */
#ifdef Make_data //制作能量机关数据集
string data_path = "/home/arno/Buff_data/";
int video_num = 4;
int frame_num = 0;
int write_num = 0;
#endif

#define Inaction_Path "./buff_inaction_new.jpg" //未激活扇叶的模板
#define Action_Path "./buff_action_new.jpg"//激活扇叶的模板
//#define Inaction_Path "./buff_inaction_s.jpg.jpg"
//#define Action_Path "./buff_action_s.jpg.jpg"
// extern int detect_mode;//1
extern  atomic<int> shootmode;//small big 静止
extern int speedmode;//30射速 18射速
extern bool atk_mode;//设计模式 常规击打 还是 4、5片连打的模式，但是由于摄像头视野太小，无法实现连打的模式，只写了大概功能，并未进行测试


Mat ForImg(Mat &img);
Mat SSEImg(Mat &img);
class Buff_Class
{
public:
    struct BUFF
    {         
        //声明结构体
        bool init_flag = false; //初始化 标志
        bool first_find =false; //第一次识别 标志
        bool clear_flag = false; //清除 标志
        bool atk_final = false; //击打最后一片扇叶 标志
        int inaction_confidence = 0; //未激活扇叶的置信度

        vector<RotatedRect> action;//已经激活的内轮廓矩形
        vector<RotatedRect> action_fan; //已经激活的外轮廓扇叶
        vector<Point2f> pts; //能量机关的四个点
        vector<Mat>temp; //能量机关模板
        vector<double>v_data; //能量机关角速度数据
        RotatedRect inaction; //已经未激活的内轮廓矩形
        RotatedRect inaction_fan;//已经未激活的外轮廓扇叶
        Point2d center;//能量机关R标
        Point2f Last_pts;//上一次目标的位置
        Point2f final_inaction;//最后一片的位置

        float angle;//能量机关角度
        float radius;//能量机关像素半径
        float last_v;//上一次速度
        float Last_angle;//上一次角度
        double last_radius;//上一次半径
        double rotations = 0.0;//避免旋转时 角度为0 突变成 -359，调节角度平缓过度
        //A * Sin(2*PI*f * t + theta) + C
        atomic<double> sin_A;//预测结果 正弦的振幅
        atomic<double> sin_f;//频率
        atomic<double> sin_theta;//相位
        atomic<double> sin_C;//平均速度
        
        vector<struct timeval> v_time;//记录当前速度的时间
        struct timeval ThisFrameTime;//这一帧的时间
        struct timeval tt1;//第一个速度的时间
        long long int pnp_time;//pnp的时间 从第四片扇叶开始时计时
    };
    struct SinData //声明结构体
    {         
        double sin_A;
        double sin_f;
        double sin_theta;
        double sin_C;

    };
    BUFF Buff;
    // SinData Sin_next; //下一次预测曲线
    bool Buff_Task(Mat &src);//能量机关任务
    Mat Pretreat_img(Mat &src);//图像预处理

    // static Mat Pretreat_img(Mat &src);
    //  Mat ForImg(Mat &img); //遍历像素的预处理
    //  Mat SSEImg(Mat &img); //使用SSE的预处理

    double getDistance_buff(CvPoint pointO, CvPoint pointA);//获取两点直接距离
    bool Find_buff(RotatedRect& light_rect,Mat &light_color,Mat& drawimg);//寻找buff
    Point2f calcPoint(Point2f center, double R, double angle);//根据Buff中心点、半径、角度，计算扇叶位置
    Point2f getCrossPoint(Point2f pt_1, double angle_1,Point2f pt_2, double angle_2); //两条直线的交点 两片扇叶的交点就是R标
    bool effective_roi(vector<Point2d> in_pt,vector<Point2d> &out_pt,Mat img);//获取合法ROI
    void Add_Pnp_buff(Point2f pre_center, bool deviate,Mat &output);//pnp
    void MissBuff(int &is_disappear, int &is_activated);//扇叶消失后事件
    int GetRectIntensity(const Mat &img, Rect rect); //计算平均灰度
    int KnowYourself(Mat &img,vector<Point2f> &points_2d_);//19年开源 辨别该扇叶是否激活
    int Match_Buff(RotatedRect Big_rect,RotatedRect Small_rect,Mat light_img,vector<Mat> temp,int Match_Mode);//扇叶模板匹配
    float Get_pts(RotatedRect Big_rect, RotatedRect Small_rect, vector<Point2f>& pts);//左上角开始 顺时针排序 坐标
    // void xcorr_fft(); //预测算法1
    // void xcorr_fft_sub_theta();//预测算法2
    void xcorr_sub();//预测算法3
    // void xcorr_fft_compare();//预测算法4
    // void conv_fft(fftw_complex *Signal_raw_out,int v_data_complex_size,double& conv_value,SinData Sin_in,SinData& Sin_out);//快速傅利叶方法的卷积
    void prediction();//预测
    void update_Vdata();//更新速度数据
    void clear_data();//清除数据
    void Buff_init();//初始化

    //XML 优化预测的速度，但是没有使用，看不懂可以不管
    double****  LoadDataXML(string FilePath);//载入XML 
    bool MakeDataXML(string FilePath);//制作XML 


private:

};
// int64_t getCurrentTime();
inline int64_t getCurrentTime()//  time/1000 = ms 获取当前时间
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000.0 *1000.0+ tv.tv_usec/1000.0;
}


#endif // BUFF_DETECT_H
