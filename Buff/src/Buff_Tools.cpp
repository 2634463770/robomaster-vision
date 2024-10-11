#include "Buff_Task.h"
// static VIDEO(SERIAL_PATH);
extern mutex mtx;

double Buff_Class::getDistance_buff(CvPoint pointO, CvPoint pointA)//两点距离
{
    double distance;
    distance = static_cast<double>(powf((pointO.x - pointA.x), 2) + powf((pointO.y - pointA.y), 2));
    distance = static_cast<double>(sqrtf(distance));
    return distance;
}
float Buff_Class::Get_pts(RotatedRect Big_rect, RotatedRect Small_rect, vector<Point2f>& pts)
{
    Point2f points[4];
    Small_rect.points(points);
    Point2f point_up_center = (points[0] + points[1])/2;
    Point2f point_down_center = (points[2] + points[3])/2;
    double up_distance = getDistance_buff(point_up_center, Big_rect.center);
    double down_distance = getDistance_buff(point_down_center, Big_rect.center);
    float angle_;
    if(up_distance > down_distance)
    {
        angle_ = -1*(Small_rect.angle-180);
        pts.push_back(points[0]);
        pts.push_back(points[1]);
        pts.push_back(points[2]);
        pts.push_back(points[3]);


    }else
    {
        angle_ = -1*Small_rect.angle;
        pts.push_back(points[2]);
        pts.push_back(points[3]);
        pts.push_back(points[0]);
        pts.push_back(points[1]);
    }
    return angle_;
}
Point2f Buff_Class::getCrossPoint(Point2f pt_1, double angle_1,Point2f pt_2, double angle_2){
    double ka, kb;
    ka = -1.0*tan(angle_1/180.0*3.1415926);
    kb = -1.0*tan(angle_2/180.0*3.1415926);
    Point2f crossPoint;
    crossPoint.x = (ka*pt_1.x - pt_1.y - kb*pt_2.x + pt_2.y) / (ka - kb);
    crossPoint.y = (ka*kb*(pt_1.x - pt_2.x) + ka*pt_2.y - kb*pt_1.y) / (ka - kb);
    return crossPoint;
}
int Buff_Class::GetRectIntensity(const Mat &img, Rect rect){
    if(rect.width < 1 || rect.height < 1 || rect.x < 1 || rect.y < 1
            || rect.width + rect.x > img.cols || rect.height + rect.y > img.rows)
        return 255;
    Mat roi = img(Range(rect.y, rect.y + rect.height), Range(rect.x, rect.x + rect.width) );
//    imshow("roi ", roi);n
    int average_intensity = static_cast<int>(mean(roi).val[0]);
    return average_intensity;
}
int Buff_Class::KnowYourself(Mat &img,vector<Point2f> &points_2d_)
{
    Point2f vector_height = points_2d_[0] - points_2d_[3];
    Point left_center = points_2d_[3] - vector_height;
    Point right_center = points_2d_[2] - vector_height;
    int width = 5;
    int height = 5;
    Point left1 = Point(left_center.x - width, left_center.y - height);
    Point left2 = Point(left_center.x + width, left_center.y + height);
    Point right1 = Point(right_center.x - width, right_center.y - height);
    Point right2 = Point(right_center.x + width, right_center.y + height);
    Rect left_rect(left1, left2);
    Rect right_rect(right1, right2);
    int left_intensity = GetRectIntensity(img, left_rect);
    int right_intensity = GetRectIntensity(img, right_rect);
//    putText(img, to_string(left_intensity), left_center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255));
//    putText(img, to_string(right_intensity), right_center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255));
//    imshow("test", img);
    if(left_intensity > 10 && right_intensity > 10)
    {
        return 1;
    }else{
        return 0;
    }
}
int Buff_Class::Match_Buff(RotatedRect Big_rect,RotatedRect Small_rect,Mat light_img,vector<Mat> temp,int Match_Mode) 
{
//    temp = imread("/home/arno/video/buff_inaction.jpg");
//    imshow("temp",temp[0]);
    //cout << Box.size.height / Box.size.width<<endl;
    //Box = minAreaRect(contours2[i]);
    if(temp.size()==0 || temp[0].empty())
        return 0;

    Big_rect.size.width *= 1.1;
    Big_rect.size.height *= 1.1;
//    vector<Point2f> P
//    Box.points(pts);
    /*
    for (int j = 0; j < 4; j++)
    {
        line(drawimg_box, pts[j], pts[(j + 1) % 4], Scalar(0, 255, 0), 2, 8);
    }*/
    Point2f points[4];
    Big_rect.points(points);
    Point2f point_up_center = (points[0] + points[3])/2;
    Point2f point_down_center = (points[1] + points[2])/2;
    double up_distance = getDistance_buff(point_up_center, Small_rect.center);
    double down_distance = getDistance_buff(point_down_center, Small_rect.center);
    vector<Point2f> srcRect;
    vector<Point2f> dstRect;
    if(up_distance < down_distance)
    {
        srcRect.push_back(points[0]);
        srcRect.push_back(points[1]);
        srcRect.push_back(points[2]);
        srcRect.push_back(points[3]);


    }else
    {
        srcRect.push_back(points[2]);
        srcRect.push_back(points[3]);
        srcRect.push_back(points[0]);
        srcRect.push_back(points[1]);
    }
    float width;
    float height;
    width = float(temp[0].cols);
    height = float(temp[0].rows);
    if(temp[0].size()!=temp[1].size())
        resize(temp[1],temp[0],temp[0].size());
//    width = getDistance_buff(srcRect[0],srcRect[1]);
//    height = getDistance_buff(srcRect[1],srcRect[2]);
    if (up_distance <= 0 || down_distance <= 0)
    {
        return -1;
    }
    dstRect.push_back(Point2f(0, 0));
    dstRect.push_back(Point2f(width, 0));
    dstRect.push_back(Point2f(width, height));
    dstRect.push_back(Point2f(0, height));

    Mat transform = getPerspectiveTransform(srcRect, dstRect);
    Mat perspectMat;
    warpPerspective(light_img, perspectMat, transform, Size(width, height), INTER_LINEAR);
    Mat testim;
    testim = perspectMat(Rect(0, 0, int(width), int(height)));
    if(Match_Mode<2 && !testim.empty())
    {
//        resize(testim, testim, temp[Match_Mode].size());
//        resize(temp[0], temp[0], Size(42, 20));
#ifdef DEBUG_IMG
            imshow("testim", perspectMat);
#endif
        int img_width = testim.cols - temp[Match_Mode].cols + 1;
        int	img_height = testim.rows - temp[Match_Mode].rows + 1;
        Mat result(img_width, img_height, CV_32FC1);
        Point temploc;
        Point minloc, maxloc;
        double min, max, loc;
        matchTemplate(testim, temp[Match_Mode], result, TM_CCOEFF_NORMED, Mat());
        minMaxLoc(result, &min, &max, &minloc, &maxloc, Mat());
        if(Match_Mode==0&&max>0.80){
            return 0;
        }
        else if (Match_Mode==1&&max>0.70) {
            return 1;
        }else {
            return -1;
        }
    }else {
        for (int i = 0;i<Match_Mode;i++) {
            if(testim.empty())
                continue;
//            resize(testim, testim, temp[i].size());
    //        resize(temp[0], temp[0], Size(42, 20));
#ifdef DEBUG_IMG
            imshow("testim", testim);
#endif
            int img_width = testim.cols - temp[i].cols + 1;
            int	img_height = testim.rows - temp[i].rows + 1;
            Mat result(img_width, img_height, CV_32FC1);
            Point temploc;
            Point minloc, maxloc;
            double min[2], max[2], loc;
            matchTemplate(testim, temp[i], result, TM_CCOEFF_NORMED, Mat());
            minMaxLoc(result, &min[i], &max[i], &minloc, &maxloc, Mat());
            Mat light_img_;
            cvtColor(light_img,light_img_,CV_GRAY2BGR);
            putText(light_img_,
                    to_string(max[i]),
                    Small_rect.center,FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,255,0));
            if(i==0&&max[i]>0.80){
                return 0;}
            else if (i==1 && max[0] >0.80 && max[1]>0.80) {
                if(max[0]>max[1])
                    return 0;
                else {
                    return 1;
                }
            }
            else if (i==1&&max[1]>0.70&max[0]<0.80) {

                return 1;
            }else if(i==1){
                return -1;
            }

        }

    }

}
Point2f Buff_Class::calcPoint(Point2f center, double R, double angle)
{
    return center + Point2f((double)cos(angle), (double)-sin(angle)) * (double)R;
}
bool Buff_Class::effective_roi(vector<Point2d> in_pt,vector<Point2d> &out_pt,Mat img){
    if(in_pt[0].x>img.cols || in_pt[0].y>img.rows || in_pt[2].x<0 || in_pt[2].y<0)
        return false;

    if(0 <= in_pt[0].x && 0 <= (in_pt[1].x-in_pt[0].x) && in_pt[1].x  <= img.cols && 0 <= in_pt[0].y && 0 <= (in_pt[3].y-in_pt[0].y) && in_pt[3].y <= img.rows){
        return true;
    }else{
        vector<Point2d>temp_pt=in_pt;
        if(in_pt[1].x>img.cols)
            temp_pt[1].x=img.cols;
        if(in_pt[2].x>img.cols)
            temp_pt[2].x=img.cols;

        if(in_pt[2].y>img.rows)
            temp_pt[2].y=img.rows;
        if(in_pt[3].y>img.rows)
            temp_pt[3].y=img.rows;

        if(in_pt[0].x<0)
            temp_pt[0].x=0;
        if(in_pt[3].x<0)
            temp_pt[3].x=0;

        if(in_pt[0].y<0)
            temp_pt[0].y=0;
        if(in_pt[1].x<0)
            temp_pt[1].y=0;
        out_pt = temp_pt;
    }
    return true;


}
void Buff_Class::MissBuff(int &is_disappear, int &is_activated)
{
        //     if(CameraMode != 1)
        //     disappear_times=60;
        // else
    // int disappear_times = 0;
    // disappear_times=15;   
    // cout<<ds.camera_pit_angle<<endl;                  
    extern vector<double> dAngle_fft;         
    if(Buff.first_find && is_disappear<=100)
    {
        extern double Curr_P,Curr_Y;
        //连续disappear_times未识别到或连续disappear_times/5*4帧该叶片激活，则沿着能量机关相反的角度移动
        //视野太小，目标可能在另一边

        if(is_disappear < disappear_times)
        {
            if(is_activated < disappear_times/5*4)
            {
                //同一片扇叶帧识别不到目标
                for (int i =0;i<Buff.action.size();i++) 
                {
                    if(getDistance_buff(Buff.Last_pts,Buff.action[i].center)<80)
                        is_activated ++;                    
                }
                Data_Get dg;
                dg.get_xy_data(0, 0, 0,0);
                serial_.send_data(dg);
            }
            else 
            {
                Data_Get dg;
                double transfer =10;  
                if((ds.camera_pit_angle)<-5.0 && (ds.camera_pit_angle)>-30.0 && is_disappear<=105){
                    //根据之前的叶片角度，云台反方向移动
                    dg.get_xy_data(static_cast<int32_t>(transfer*cos(Buff.angle/180*PI) * 1000.0 * -1.0), 
                    static_cast<int32_t>(transfer*sin(Buff.angle/180*PI) * 1000.0 * 1.0), 1,0);
                }
                else
                    dg.get_xy_data(0, 0, 0,0);
                serial_.send_data(dg);

            }
        }
        else 
        {
            Data_Get dg;
            double transfer =10;
            if((ds.camera_pit_angle)<-5.0 && (ds.camera_pit_angle)>-30.0 && is_disappear<=105){

                //根据之前的叶片角度，云台反方向移动
                dg.get_xy_data(static_cast<int32_t>(transfer*cos(Buff.angle/180*PI) * 1000.0 * -1.0), static_cast<int32_t>(transfer*sin(Buff.angle/180*PI) * 1000.0 * 1.0), 1,0);

            }
            else
            {
                dg.get_xy_data(0, 0, 0, 0);
            }
            serial_.send_data(dg);
        }
        is_disappear ++;
        if(Buff.v_data.size()>35 && shootmode ==1)
        {
            if(is_disappear < 50 && is_activated<40)
            {
                dAngle_fft.push_back(1.305);
                if(dAngle_fft.size()%num_mean==0)
                {
                    double dAngle_sum = 0;
                    for (int j=0;j<num_mean;j++) {
                        dAngle_sum = dAngle_sum + dAngle_fft[j];
                    }
                    dAngle_fft.clear();
                    dAngle_fft.shrink_to_fit();
                    mtx.lock();
                    Buff.v_data.push_back(dAngle_sum/180*PI*fs/num_mean);
                    // Buff.v_time.push_back(getCurrentTime());
                    // Buff.v_data.push_back(1.305);
                    //Change Time;
                    Buff.v_time.push_back(Buff.ThisFrameTime);
                    mtx.unlock();
                }
            
            }
            else 
            {
                    dAngle_fft.clear();
                    dAngle_fft.shrink_to_fit();
                    mtx.lock();
                    Buff.v_data.clear();
                    Buff.v_data.shrink_to_fit();
                    Buff.v_time.clear();
                    Buff.v_time.shrink_to_fit();
                    mtx.unlock();
            }

        }
        else if (Buff.v_data.size()<=35 && shootmode ==1) 
        {
            if(is_disappear > 50 || is_activated>40)
            {
                //消失时间过长，清除数据，重新计算
                dAngle_fft.clear();
                dAngle_fft.shrink_to_fit();
                mtx.lock();
                Buff.v_data.clear();
                Buff.v_data.shrink_to_fit();
                Buff.v_time.clear();
                Buff.v_time.shrink_to_fit();
                mtx.unlock();
            }
            else 
            {
                //时间不长，则默认速度为1.305
                dAngle_fft.push_back(1.305);
                if(dAngle_fft.size()%num_mean)
                {
                    double dAngle_sum = 0;
                    for (int j=0;j<num_mean;j++) {
                        dAngle_sum = dAngle_sum + dAngle_fft[j];
                    }
                    dAngle_fft.clear();
                    dAngle_fft.shrink_to_fit();
                    mtx.lock();
                    Buff.v_data.push_back(dAngle_sum/180*PI*fs/num_mean);
                    // Buff.v_time.push_back(getCurrentTime());
                    Buff.v_time.push_back(Buff.ThisFrameTime);
                    mtx.unlock();
                }
            }
        }
    }
    else 
    {
        Data_Get dg;
        dg.get_xy_data(0, 0, 0,0);
        serial_.send_data(dg);
    }

}