#include "Buff_Task.h"
// extern int Detect_Mode;//1
extern  atomic<int> shootmode;
extern int speedmode;//30射速 18射速
extern double Buff_R;
extern int fire_flag;
extern bool Curr_fire_flag;
extern int fire_confidence;
extern int change_confidence;
extern double add_predict_time;
extern double add_t0_time;
extern double buff_pitch_up_down;
extern double buff_yaw_left_right;
// static VIDEO(SERIAL_PATH);
extern int V_direction;//1=ringht -1=left 顺时针1 逆时针-11 未知0
extern mutex mtx;//互斥锁

vector<RotatedRect> undetermined_inaction_target; //待确定的未激活目标
vector<RotatedRect> undetermined_inaction_target_fan;//待确定的未激活扇叶
vector<RotatedRect> undetermined_inaction_target_last;//上一次待确定的未激活目标
vector<int> undetermined_inaction_confidence;//待确定的未激活目标置信度
vector<int> undetermined_inaction_confidence_last;//上一次待确定的未激活目标置信度
Mat Buff_Class::Pretreat_img(Mat &src)//图像预处理
{
    GaussianBlur(src, src, Size(3,3),0);
    Mat mask;
    Mat gray;
    cvtColor(src, gray, CV_BGR2GRAY);
    if(hsv) //使用hsv
    {
        Mat hsv_img;
        if(BLUE)
        {
            Scalar color1 = Scalar(0, 0, 255);
            Scalar color2 = Scalar(42, 74, 255);
            cvtColor(src, hsv_img, CV_BGR2HSV);
            inRange(hsv_img, color1, color2, mask);
        }
        else
        {
            Scalar color1 = Scalar(0, 0, 255);
            Scalar color2 = Scalar(42, 74, 255);       //亮 红
//            Scalar color1 = Scalar(0, 32, 104);
//            Scalar color2 = Scalar(88, 255, 255);      //暗 红
            cvtColor(src, hsv_img, CV_BGR2HSV);
            inRange(hsv_img, color1, color2, mask);
        }
    }
    else
    {
        //BGR颜色通道相减
        vector<Mat>bgr;
        Mat Light_color;
        split(src, bgr);
#ifdef DEBUG_MODE
        bool DectionColor = BUFF_is_BLUE;
#else
        bool DectionColor = !ds.car_color;  
#endif

        if(ds.car_color==1)
        {
            subtract(bgr[0], bgr[2], Light_color);//颜色相减 获得灯条颜色区域
            threshold(Light_color, mask, Blue_Value, 255, CV_THRESH_BINARY);
            threshold(gray, gray, Blue_Gray_Value, 255, THRESH_BINARY);

        }
        else
        {
            subtract(bgr[2], bgr[1], Light_color);//颜色相减 获得灯条颜色区域
            threshold(Light_color, mask, Red_Value, 255, CV_THRESH_BINARY);
            threshold(gray, gray, Red_Gray_Value, 255, THRESH_BINARY);


        }
        vector<Mat>().swap(bgr);
        Light_color.release();
    }
    Mat Mat_Result = mask & gray;
    Mat element = getStructuringElement(MORPH_RECT, Size(3,3));//
    // dilate(Mat_Result, Mat_Result, element);//白大
    // morphologyEx(Mat_light_color,Mat_light_color,MORPH_CLOSE,element);
#ifdef DEBUG_PIC
    // imshow("Mat_Result", Mat_Result);
    // imshow("gray", gray);
    // imshow("mask", mask);
#endif
    mask.release();
    gray.release();
    element.release();
    return Mat_Result;
}


bool Buff_Class::Find_buff(RotatedRect& light_rect,Mat &light_color,Mat& drawimg)
{
    undetermined_inaction_target.clear();
    undetermined_inaction_target.shrink_to_fit();
    undetermined_inaction_confidence.clear();
    undetermined_inaction_confidence.shrink_to_fit();
    undetermined_inaction_target_fan.clear();
    undetermined_inaction_target_fan.shrink_to_fit();
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    RotatedRect big_rect,small_rect;
    bool is_Found = false;
    findContours(light_color, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);
    vector<vector<Point>> contours_fan; 
    Point2f mc;
    Point2f MassCenter; 
    if (contours.size() > 0)
    {
        for (int i = 0; i < contours.size(); i++)
        {
            if (hierarchy[i][3] != -1)
            {
                // if(contours[hierarchy[i][3]].size() > 6)
                    // ellipse(drawimg,fitEllipse(contours[hierarchy[i][3]]),Scalar(0,255,255),2);
                //drawContours(test5, contours, i, Scalar(0, 0, 255), 1, LINE_AA, hierarchy, 0, Point(0, 0));
                // 用于寻找小轮廓，没有父轮廓的跳过, 以及不满足6点拟合椭圆
                if(hierarchy[i][3]<0 || contours[i].size() < 6 || contours[static_cast<uint>(hierarchy[i][3])].size() < 6)
                    continue;
                // 小轮廓面积条件
                double small_rect_area = contourArea(contours[i]);
//                double Light_Contour_Area = contourArea(contours[i]);//计算轮廓面积
                if(small_rect_area<5.0)continue;
                double small_rect_length = arcLength(contours[i],true);
                if(small_rect_length < 10)continue;
                double big_rect_area = contourArea(contours[static_cast<uint>(hierarchy[i][3])]);
                if(big_rect_area < 300) continue;
                double big_rect_length = arcLength(contours[static_cast<uint>(hierarchy[i][3])],true);
                if(big_rect_length < 50)continue;
//                small_rect = minAreaRect(contours[i]);
//                RotatedRect big_rect = minAreaRect(contours[hierarchy[i][3]]);
                RotatedRect big_rect = fitEllipse(contours[hierarchy[i][3]]);
                small_rect = fitEllipse(contours[i]);

                double small_rect_width = small_rect.size.width, small_rect_height = small_rect.size.height;
                double big_rect_width = big_rect.size.width, big_rect_height = big_rect.size.height;
                if (small_rect_height > small_rect_width)
                    swap(small_rect_height, small_rect_width);
                if (big_rect_height < big_rect_width)
                    swap(big_rect_height, big_rect_width);
//                float diff_angle=fabsf(big_rect.angle-small_rect.angle);
//                ellipse(drawimg,big_rect,Scalar(0,255,0),3);
                float diff_angle=fabsf(big_rect.angle-small_rect.angle);
                if(small_rect.size.height/small_rect.size.width < 3)
                {

                    if(diff_angle>70 && diff_angle<110 )
                    {

                        float small_rect_size_ratio;
                        small_rect_size_ratio = small_rect.size.height/small_rect.size.width;
                        // 根据轮廓面积进行判断扇叶类型
                        if(contourArea(contours[hierarchy[i][3]])/small_rect.size.area() > 2.3 && small_rect_area * 12 >big_rect_area
                                && small_rect_area* area_ratio<big_rect_area && small_rect_size_ratio > 1.2 && small_rect_size_ratio < 3.0f
                                && (big_rect_length/small_rect_length)/(big_rect_area/small_rect_area) < 0.6
                                && ((big_rect_length/small_rect_length)*(big_rect_area/small_rect_area)/(big_rect_height/small_rect_height)) > 2.5)
                        {
                            
                            Buff.action.push_back(small_rect);
                            Buff.action_fan.push_back(big_rect);
                            contours_fan.push_back(contours[hierarchy[i][3]]);
                        
                        }
                        bool Detectcondition0 = small_rect_area * area_ratio >= big_rect_area;
                        bool Detectcondition1 = small_rect_area * 1.6 < big_rect_area;
                        bool Detectcondition2 = small_rect_size_ratio > 1.0;                        
                        bool Detectcondition3 = small_rect_size_ratio < 3.0f;
                        bool Detectcondition4 = (big_rect_length/small_rect_length)/(big_rect_area/small_rect_area) >= 0.50;//0.55
                        bool Detectcondition5 = ((big_rect_length/small_rect_length)*(big_rect_area/small_rect_area)/(big_rect_height/small_rect_height)) < 3.2;
#ifdef DEBUG_PIC        
                        if(ui_open)
                        {                            
                            char Condition0_Text[20];
                            char Condition1_Text[20];
                            char Condition2_Text[20];
                            char Condition3_Text[20];
                            char Condition4_Text[20];
                            char Condition5_Text[20];

                            sprintf(Condition0_Text, "0: %.2f!>=%.2f ", big_rect_area/small_rect_area,area_ratio);
                            sprintf(Condition1_Text, "1: %.2f!<1.6 ", big_rect_area/small_rect_area);
                            sprintf(Condition2_Text, "2: %.2f!>1.0 ", small_rect_size_ratio);
                            sprintf(Condition3_Text, "3: %.2f!<3.0 ", small_rect_size_ratio);
                            sprintf(Condition4_Text, "4: %.2f!>=0.50 ", (big_rect_length/small_rect_length)/(big_rect_area/small_rect_area));
                            sprintf(Condition5_Text, "5: %.2f!>=0.32", ((big_rect_length/small_rect_length)*(big_rect_area/small_rect_area)
                            /(big_rect_height/small_rect_height)));

                            if(big_rect_area/small_rect_area<12.0)
                            putText(drawimg,
                            (Detectcondition0 ? "":String(Condition0_Text))+
                            (Detectcondition1 ? "":String(Condition1_Text))+
                            (Detectcondition2 ? "":String(Condition2_Text))+
                            (Detectcondition3 ? "":String(Condition3_Text))+
                            (Detectcondition4 ? "":String(Condition4_Text))+
                            (Detectcondition5 ? "":String(Condition5_Text))
                            ,small_rect.center,FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,255,0));
                        }
#endif
                        if(
                            Detectcondition0 && 
                            Detectcondition1 &&
                            Detectcondition2 && 
                            Detectcondition3 && 
                            Detectcondition4 &&
                            Detectcondition5
                            )
                        {

                            // Buff.angle = Get_pts(big_rect, small_rect, Buff.pts);
                            vector<Point2f> temp_pts;
                            Get_pts(big_rect, small_rect, temp_pts);
                            if(KnowYourself(light_color,temp_pts))
                            {
                                // if(Buff.temp.size()!=0 && !Buff.temp[0].empty()) //模板导入正常
                                // {

                                //     if(Match_Buff(big_rect,small_rect,light_color,Buff.temp,1)==1)//模版匹配 
                                //     {                                    
                                //         Buff.action.push_back(small_rect);                                    
                                //         Buff.action_fan.push_back(big_rect);
                                //         contours_fan.push_back(contours[hierarchy[i][3]]);
                                //         continue;
                                //     }
                                //     else 
                                //     {
                                //         light_rect=small_rect;
                                //         contours_fan.push_back(contours[hierarchy[i][3]]);

                                //     }

                                // }else
                                // {
                                    Buff.action.push_back(small_rect);                                    
                                    Buff.action_fan.push_back(big_rect);
                                    contours_fan.push_back(contours[hierarchy[i][3]]);
                                    continue;

                                // }
                            }
                            else 
                            {
                                light_rect=small_rect;
                                contours_fan.push_back(contours[hierarchy[i][3]]);
                            }

                            //计算轮廓矩
                            Moments mu = moments(contours[i], false);
                            //计算轮廓的质心
                            mc = Point2d(mu.m10/mu.m00 , mu.m01/mu.m00);
                            // circle(drawimg,mc,3,Scalar(0,255,255),-1);
                            Moments mu1 = moments(contours[hierarchy[i][3]], false);
                            MassCenter = Point2d(mu1.m10/mu1.m00 , mu1.m01/mu1.m00); //质心

                            undetermined_inaction_confidence.push_back(0);
                            undetermined_inaction_target.push_back(light_rect);
                            undetermined_inaction_target_fan.push_back(big_rect);
                            is_Found=true;//寻找到目标

                        }
                        else
                        {
//                             putText(drawimg,to_string(i),small_rect.center,FONT_HERSHEY_SIMPLEX,0.2,Scalar(255,255,0));
                        }
                    }
                }
//            }

            }
        }
    }
    vector<vector<Point>>().swap(contours);//释放内存
    vector<Vec4i>().swap(hierarchy);
    if(is_Found)
    {
        //根据两次数据，匹配叶片
        for(int i = 0;i<undetermined_inaction_target.size();i++)
        {
            for(int j = 0;j<undetermined_inaction_target_last.size();j++)
            {
                double max_length = max(undetermined_inaction_target[i].size.width,undetermined_inaction_target[i].size.height);
                //两者直接的距离差距不大，则认为是同一片扇叶
                if (fabs(undetermined_inaction_target_last[j].center.x - undetermined_inaction_target[i].center.x) < max_length 
                &&  fabs(undetermined_inaction_target_last[j].center.y - undetermined_inaction_target[i].center.y) < max_length)
                // ||abs((360 * Buff.rotations + Buff.angle - Buff.Last_angle)) < 25.0)
                {
                    undetermined_inaction_confidence[i] = undetermined_inaction_confidence_last[j];
                    undetermined_inaction_confidence[i] = undetermined_inaction_confidence[i] + 1;//上次的+1
                }
            }

        }

        int Curr_Confidence = undetermined_inaction_confidence[0];
        light_rect = undetermined_inaction_target[0];
        Buff.inaction = undetermined_inaction_target[0];
        Buff.inaction_fan = undetermined_inaction_target_fan[0];
        //从未激活变成已激活状态有延迟，会出现两片未激活扇叶
        if(undetermined_inaction_target.size()>1)//若是出现两片未激活扇叶
        {
            
            for(int i = 1;i<undetermined_inaction_target.size();i++)
            {
#ifdef DEBUG_PIC
                if(ui_open)
                { 
                    putText(drawimg,to_string(undetermined_inaction_confidence[i]),undetermined_inaction_target[i].center,FONT_HERSHEY_SIMPLEX,1,Scalar(255,255,0));
                    putText(drawimg,to_string(undetermined_inaction_confidence[0]),undetermined_inaction_target[0].center,FONT_HERSHEY_SIMPLEX,1,Scalar(0,255,255));
                }
#endif
                if(Curr_Confidence>undetermined_inaction_confidence[i] && undetermined_inaction_confidence[i] > 5)//选择最新出现且出现连续5帧的未激活扇叶
                {
                    Curr_Confidence = undetermined_inaction_confidence[i];
                    light_rect = undetermined_inaction_target[i];
                    Buff.inaction = undetermined_inaction_target[i];
                    Buff.inaction_fan = undetermined_inaction_target_fan[i];

                }
            }

        }
        Buff.inaction_confidence = Curr_Confidence;
        double max_length = max(Buff.inaction.size.width,Buff.inaction.size.height);

        if (fabs(Buff.Last_pts.x - Buff.inaction.center.x) > max_length || fabs(Buff.Last_pts.y - Buff.inaction.center.y) > max_length)//不是同一片扇叶
        {
            fire_confidence=0;
            if(Buff.inaction_confidence>=5)
                Curr_fire_flag=false;
            else
                Curr_fire_flag=true;
        }
        else
        {
            if(Buff.inaction_confidence==5)
                Curr_fire_flag=false;
            else if(Buff.inaction_confidence<5)
            {
                Curr_fire_flag=true;
                fire_confidence = 0;
            }


        }
        Buff.angle = Get_pts(Buff.inaction_fan, Buff.inaction, Buff.pts);
        undetermined_inaction_confidence_last.swap(undetermined_inaction_confidence);
        undetermined_inaction_target_last.swap(undetermined_inaction_target);

        double x_pt=0,y_pt=0,c_r=0;
        //根据所有扇叶，计算交点，获取R标位置
        for (int i = 0;Buff.action.size()>0 && i<Buff.action.size()+1;i++) {
            for (int j = i+1;j<Buff.action.size()+1;j++) {
                Point2f Cross_i;

                if(j==Buff.action.size()){
                    vector<Point2f>temp_pt;
                    double angle_ =Get_pts(Buff.action_fan[i],Buff.action[i],temp_pt);
                    Cross_i = getCrossPoint(Buff.inaction.center,Buff.angle,Buff.action[i].center,angle_);//两条直线的交点
                    vector<Point2f>().swap(temp_pt);

                }
                else{
                vector<Point2f>temp_pt_1;
                double angle_1 =Get_pts(Buff.action_fan[i],Buff.action[i],temp_pt_1);
                vector<Point2f>temp_pt_2;
                double angle_2 =Get_pts(Buff.action_fan[j],Buff.action[j],temp_pt_2);
                Cross_i = getCrossPoint(Buff.action[i].center,angle_1,Buff.action[j].center,angle_2);
                vector<Point2f>().swap(temp_pt_1);
                vector<Point2f>().swap(temp_pt_2);
                }

                x_pt+=Cross_i.x;
                y_pt+=Cross_i.y;
            }
        }
        Point2f CrossPt;
        if(Buff.action.size()==0){
FindCenterAgain:            
            if(Buff.last_radius > 0.001 && abs(Buff.last_radius - max(Buff.inaction.size.width, Buff.inaction.size.height) / 260 * 700 * 0.86)//R标大概位置
                    < max(Buff.inaction.size.width, Buff.inaction.size.height) * 0.3){
                CrossPt =  Point2f(Buff.inaction.center.x - Buff.last_radius * cosf(Buff.angle / 180.0f * 3.1415),
                                   Buff.inaction.center.y + Buff.last_radius * sinf(Buff.angle / 180.0f * 3.1415));
            }else 
            {
                CrossPt =  Point2f(Buff.inaction.center.x - max(Buff.inaction.size.width, Buff.inaction.size.height) / 260 * 700 * 0.86 * cosf(Buff.angle / 180.0f * 3.1415),
                                   Buff.inaction.center.y + max(Buff.inaction.size.width, Buff.inaction.size.height) / 260 * 700 * 0.86 * sinf(Buff.angle / 180.0f * 3.1415));
            }
        }else {
            CrossPt=Point2f((x_pt)/double(Buff.action.size()+1)/double(Buff.action.size())*2.0,(y_pt)/double(Buff.action.size()+1)/double(Buff.action.size())*2.0);
            if(getDistance_buff(CrossPt,Buff.inaction.center)/min(Buff.inaction.size.height,Buff.inaction.size.width)>7.0)//半径存在异常，重新计算ROI位置，寻找中心
                goto FindCenterAgain;
        }

        double len=max(Buff.inaction.size.width,Buff.inaction.size.height)*0.8;
        vector<Point2d>roi_pt;
        //初定的ROI
        roi_pt.push_back(Point2d(CrossPt.x-len,CrossPt.y-len));
        roi_pt.push_back(Point2d(CrossPt.x+len,CrossPt.y-len));
        roi_pt.push_back(Point2d(CrossPt.x+len,CrossPt.y+len));
        roi_pt.push_back(Point2d(CrossPt.x-len,CrossPt.y+len));
        bool find_center =false;
        if(effective_roi(roi_pt,roi_pt,drawimg))//计算合法的ROI，且其是有效的 
        {
            vector<vector<Point>> contours_roi;
            vector<Vec4i> hierarchy_roi;
            Mat imageROI = light_color(Rect(roi_pt[0],roi_pt[2]));
            findContours(imageROI, contours_roi, hierarchy_roi, RETR_CCOMP, CHAIN_APPROX_NONE);
            double min_dist=drawimg.cols;
            //寻找R标
            for (int i = 0;i < contours_roi.size();i++) 
            {
                if(contours_roi[i].size()<5)
                    continue;
                RotatedRect center_rect = fitEllipse(contours_roi[i]);
                double min_length = min(center_rect.size.width,center_rect.size.height), max_length = max(center_rect.size.width,center_rect.size.height);
                if(min_length/max_length< 0.4)
                    continue;
                if(center_rect.size.area()/Buff.inaction.size.area()>0.5 || center_rect.size.area()/Buff.inaction.size.area()<0.1)
                    continue;
                //计算轮廓矩
                Moments roi_mu = moments(contours_roi[i], false);
                //计算轮廓的质心
                Point2d roi_mc = Point2d(roi_mu.m10/roi_mu.m00 , roi_mu.m01/roi_mu.m00);
                Point2d center_pt = Point2d(roi_mc.x+roi_pt[0].x,roi_mc.y+roi_pt[0].y);

                // Point2d center_pt = Point2d(center_rect.center.x+roi_pt[0].x,center_rect.center.y+roi_pt[0].y);
                double center_rect_dist =getDistance_buff(center_pt,CrossPt);
                bool center_flag =false;
                for (int j = 0;j<contours_fan.size();j++) {

                    if(pointPolygonTest(contours_fan[j],center_pt,true)>-0.1*len)
                        center_flag=true;
                }
                if(center_flag)
                    continue;
                if(center_rect_dist/len>0.9)
                    continue;

                if(center_rect_dist<=min_dist){//距离中ROI中心最近
                    min_dist = center_rect_dist;
                    Point2f pt_temp[4];
                    center_rect.points(pt_temp);
                    Buff.center=center_pt;
//                    circle(drawimg,center_pt,3,Scalar(255,0,255),2);
                    find_center =true;
                    // Buff.inaction.center 矩形中心
                    // mc.y 矩形质心
                    // Buff.center R标位置 能量机关中心
                    // MassCenter.y 未激活扇叶轮廓质心 最稳定

                    // double Buff_angle = -1.0*atan2((Buff.inaction.center.y-Buff.center.y),(Buff.inaction.center.x-Buff.center.x))/PI*180.0;
                    // double Buff_angle1 = -1.0*atan2((mc.y-Buff.center.y),(mc.x-Buff.center.x))/PI*180.0;
                    // double Buff_angle2 = -1.0*atan2((MassCenter.y-Buff.center.y),(MassCenter.x-Buff.center.x))/PI*180.0;
                    double Buff_angle3 = -1.0*atan2((mc.y-MassCenter.y),(mc.x-MassCenter.x))/PI*180.0;
                    // cout<<Buff.angle<<" "<<Buff_angle<<" "<<Buff_angle1<<" "<<Buff_angle2<<" "<<Buff_angle3<<endl;
                    double Buff_angle = -1.0*atan2((MassCenter.y-Buff.center.y),(MassCenter.x-Buff.center.x))/PI*180.0;
                    //如果角度差过大，则存在异常，获取更合适的角度
                    if(abs(Buff_angle3-Buff_angle)<5.0)
                        Buff.angle=Buff_angle;
                    else if (Buff.angle>170.0 && Buff_angle<-170.0 && abs(360.0-Buff.angle+Buff_angle) <5.0) {
                        Buff.angle=Buff_angle;
                    }
                    else if (Buff.angle<-170.0 && Buff_angle>170.0 && abs(360.0-Buff.angle+Buff_angle) <5.0) {
                        Buff.angle= Buff_angle;
                    }else
                        Buff.angle = Buff_angle3;
                    // free(pt_temp);//free()函数仅用于释放通常通过malloc / calloc / realloc获得的动态分配的内存。  Error

                }
            }

            vector<vector<Point>>().swap(contours_roi);
            vector<Vec4i>().swap(hierarchy_roi);

        }
        if(!find_center){
            Buff.center = CrossPt;
            Buff.last_radius = 0.0;
        }else {
            Buff.last_radius = getDistance_buff(Buff.center,Buff.inaction.center);
        }
        Buff.radius=getDistance_buff(Buff.center,Buff.inaction.center);

        //计算最后一片扇叶位置
        for (int j = 1;j < 5 && Buff.action.size()==3 && Buff.atk_final;j++) 
        {
            bool is_pt =true;
            for (int k = 0;k < Buff.action.size();k++) {
                vector<Point2f>temp_pt;
                double action_angle =Get_pts(Buff.action_fan[k],Buff.action[k],temp_pt);
                if(getDistance_buff(Buff.action[k].center,calcPoint(Buff.center,getDistance_buff(Buff.center,Buff.inaction.center),(Buff.angle+double(j)*360.0/5.0)/180.0*PI))<
                        max(Buff.action[k].size.width,Buff.action[k].size.height)*1.5 || abs(action_angle - Buff.angle)<15)
                {
                    is_pt =false;
                }
                vector<Point2f>().swap(temp_pt);
            }
            if(is_pt){
            circle(drawimg,calcPoint(Buff.center,getDistance_buff(Buff.center,Buff.inaction.center),(Buff.angle+double(j)*360.0/5.0)/180.0*PI),3,Scalar(0,255,255),-1);
            Buff.final_inaction=calcPoint(Buff.center,getDistance_buff(Buff.center,Buff.inaction.center),(Buff.angle+double(j)*360.0/5.0)/180.0*PI);
            }
        }


#ifdef Make_Data //制作能量机关数据集，看不懂可以不管
        vector<Point2d>inaction_data;
        vector<vector<Point2d>>action_data;
        if(Buff.action.size()>0 && write_num % 10 ==0){
            write_num = 0;
            char lab_ch[30],lab_folder_ch[30];
            string lab_folder_path = data_path+"labels/"+to_string(video_num);
            strcpy(lab_folder_ch,lab_folder_path.c_str());

            if(access(lab_folder_ch,0)==-1){
                  mkdir(lab_folder_ch,S_IRWXU);
            }
            string lab_path = data_path+"labels/"+to_string(video_num)+"/"+to_string(video_num)+"_"+to_string(frame_num)+".txt";
            strcpy(lab_ch,lab_path.c_str());
            FILE *fp;
            if(fp = fopen(lab_ch,"w+"))
            {
                cout<<"can"<<endl;
            }else {
                cout<<"not"<<endl;
            }
            inaction_data.push_back(Buff.center);
            inaction_data.push_back(Buff.pts[3]);
            inaction_data.push_back(Buff.pts[0]);
            inaction_data.push_back(Buff.pts[1]);
            inaction_data.push_back(Buff.pts[2]);
            for (int j =0;j<Buff.action.size();j++) {
    //            Point2f pts[4];
    //            Buff.action[i].points(pts);
    //            for (int j = 0;j<4;j++) {
    //                line(src,pts[j],pts[(j+1)%4],Scalar(255,120,255),2);
    //            }
                vector<Point2f> action_pts;
                Get_pts(Buff.action_fan[j],Buff.action[j],action_pts);
                vector<Point2d> action_temp;
                action_temp.push_back(Point2d(Buff.center));
                action_temp.push_back(action_pts[3]);
                action_temp.push_back(action_pts[0]);
                action_temp.push_back(action_pts[1]);
                action_temp.push_back(action_pts[2]);
                action_data.push_back(action_temp);

    //            for (int j = 0;j<4;j++) {
    //                line(src,action_pts[j],action_pts[(j+1)%4],Scalar(255,120,255),2);
    //                putText(src, to_string(j), action_pts[j], 3, 1.0, Scalar(255, 0, 0));
    //            }

//                for (int j = 0;j<5;j++) {
//                    line(src,action_data[i][j],action_data[i][(j+1)%5],Scalar(255,120,255),2);
//                    putText(src, to_string(j), action_data[i][j], 3, 1.0, Scalar(255, 0, 0));
//                }
                fprintf(fp, "0 %f %f ", action_data[j][0].x/drawimg.cols,action_data[j][0].y/drawimg.rows);
                for (int k = 1;k<4;k++) {
                fprintf(fp, "%f %f ", action_data[j][k].x/drawimg.cols,action_data[j][k].y/drawimg.rows);
                }
                fprintf(fp, "%f %f\n", action_data[j][4].x/drawimg.cols,action_data[j][4].y/drawimg.rows);
            }
            fprintf(fp, "1 %f %f ", inaction_data[0].x/drawimg.cols,inaction_data[0].y/drawimg.rows);
            for (int j = 1;j<4;j++) {
                fprintf(fp, "%f %f ", inaction_data[j].x/drawimg.cols,inaction_data[j].y/drawimg.rows);
            }
            fprintf(fp, "%f %f\n", inaction_data[4].x/drawimg.cols,inaction_data[4].y/drawimg.rows);
//            for (int j = 0;j<5 && Buff.action.size()>0;j++) {
//                line(src,inaction_data[j],inaction_data[(j+1)%5],Scalar(255,120,255),2);
//                putText(src, to_string(j), inaction_data[j], 3, 1.0, Scalar(255, 0, 0));
//            }

            char img_folder_ch[30];
            string img_folder_path = data_path+"images/"+to_string(video_num);
            strcpy(img_folder_ch,img_folder_path.c_str());
            if(access(img_folder_ch,0)==-1){
                  mkdir(img_folder_ch,S_IRWXU);
            }
            string img_path = data_path+"images/"+to_string(video_num)+"/"+to_string(video_num)+"_"+to_string(frame_num)+".jpg";
            imwrite(img_path,drawimg);
            inaction_data.clear();
            action_data.clear();
            frame_num++;
            fclose(fp);
            for (int j = 0;j<4;j++) {
                line(drawimg,Buff.pts[j],Buff.pts[(j+1)%4],Scalar(255,120,255),2);
            }
        }else if (Buff.action.size()>0) {
            write_num++;
        }
#endif
#ifdef DEBUG_PIC
        if(ui_open)
        { 
            for (int i =0;i<4;i++) {
                line(drawimg,roi_pt[i],roi_pt[(i+1)%4],Scalar(0,255,255),1);
            }
            line(drawimg,Buff.center,mc,Scalar(0,255,0),3);

            for (int i =0;i<5;i++) {
                circle(drawimg,calcPoint(Buff.center,getDistance_buff(Buff.center,mc),(Buff.angle+double(i)*360.0/5.0)/180.0*PI),3,Scalar(0,255,255),-1);
            }
            circle(drawimg,MassCenter,5,Scalar(255,255,0),-1);
        }

#endif
        vector<Point2d>().swap(roi_pt);

    }
    vector<vector<Point>>().swap(contours_fan);
    return is_Found;
}


    
Mat ForImg(Mat &img) //遍历像素的预处理 没有调用 可以不管
{

    GaussianBlur(img, img, Size(3,3),0);
    Mat dilated_my;
    dilated_my.create(img.rows, img.cols, CV_8UC1);
    Mat dst = Mat::ones(img.size(), CV_8UC1);
    int channels = img.channels();
    omp_set_num_threads(8);
#pragma omp parallel for
    for (int row = 0; row < img.rows; ++row)
    {
        // if(omp_get_thread_num()!=0)
        // cout << omp_get_thread_num() << endl;
        uchar* PixelPointer = NULL;
        uchar* RawPointer = NULL;
        PixelPointer = dst.ptr<uchar>(row);
        RawPointer = img.ptr<uchar>(row);
        for (register int col = 0; col < img.cols; col++)
        {	

            if (BUFF_is_BLUE)
            {
                register uchar GrayVal = (((float)*(RawPointer + col * channels + 2) * 0.299 + (float)*(RawPointer + col * channels + 1) * 0.587 +
                    (float)*(RawPointer + col * channels) * 0.114) >= Blue_Gray_Value ? 255 : 0);
                (*(RawPointer + col * channels) - *(RawPointer + 2 + col * channels)) > Blue_Value ?
                    *(PixelPointer + col) = (255 & GrayVal) : *(PixelPointer + col) = 0;

            }
            else
            {
                register uchar GrayVal = (((float)*(RawPointer + col * channels + 2) * 0.299 + (float)*(RawPointer + col * channels + 1) * 0.587 +
                    (float)*(RawPointer + col * channels) * 0.114) >= Red_Gray_Value ? 255 : 0);
                (*(RawPointer + 2 + col * channels) - *(RawPointer + col * channels)) > Red_Value ?
                    *(PixelPointer + col) = (255 & GrayVal) : *(PixelPointer + col) = 0;
            }
            //uchar minV = 255;
            //uchar maxV = 0;
            //遍历周围最大像素值
#ifdef Mat_Dilate
            for (int yi = row - 1; yi <= row + 1; yi++)
            {
                uchar* PixelValue = NULL;
                uchar* PixelValue1 = NULL;
                if (yi >= 0 && yi < img.rows) {
                    PixelValue = img.ptr<uchar>(yi);
                    PixelValue1 = dst.ptr<uchar>(yi);
                }
                for (int xi = col - 1; xi <= col + 1; xi++)
                {
                    if (xi < 0 || xi >= img.cols || yi < 0 || yi >= img.rows)
                    {
                        continue;
                    }
                    //minV = (std::min<uchar>)(minV, img.at<uchar>(yi, xi));
                    if (*(PixelValue1 + xi) == 1)
                        if (is_Blue)
                            maxV = (std::max<uchar>)(maxV, (*(PixelValue + xi * channels) - *(PixelValue + xi * channels + 2) > Threshold_Value ? 255 : 0));
                        else
                            maxV = (std::max<uchar>)(maxV, (*(PixelValue + 2 + xi * channels) - *(PixelValue + xi * channels) > Threshold_Value ? 255 : 0));
                    else
                        maxV = (std::max<uchar>)(maxV, *(PixelValue1 + xi));

                }
            }
            dilated_my.at<uchar>(row, col) = maxV;
#endif // Mat_Dilate
        }
    }
    // Mat element = getStructuringElement(MORPH_RECT, Size(3,3));//
    // dilate(dst, dst, element);//白大
    return dst;
}
// Mat Buff_Class::SSEImg(Mat &img)
Mat SSEImg(Mat &img) //使用SSE的预处理 没有调用 可以不管
{
    omp_set_num_threads(8);

    GaussianBlur(img, img, Size(3,3),0);

    vector<Mat>SSE_BGR;
    split(img, SSE_BGR);
    Mat OutImg = Mat::zeros(img.size(), CV_8UC1);
    Mat GrayImg = Mat::zeros(img.size(), CV_8UC1);
    //unsigned char nThreshold = Threshold_Value + 1;// (PixelValue > Threshold_Value) == (PixelValue > = Threshold_Value + 1)
    register unsigned char GrayThreshold = Blue_Gray_Value + 1;// (PixelValue > Threshold_Value) == (PixelValue > = Threshold_Value + 1)
    //申请的变量是存储在CPU中央寄存器中的（寄存器是中央处理器内的组成部分。寄存器是有限存贮容量的高速存贮部件），而使用int申请的变量是存储在内存中。
    //使用register修饰的变量可以提高它的读写速度，一般用于多层循环中。
    //OutImg = Mat(InImg.rows, InImg.cols, CV_8UC1);
    register unsigned char ColorThreshold = Blue_Value + 1;// (PixelValue > Threshold_Value) == (PixelValue > = Threshold_Value + 1)

    int Width = img.cols;
    int Height = img.rows;
    const int BlockSize = 16;
    int Block = Width / BlockSize;
    register int Color1 = 0, Color2 = 0;
    // BUFF_is_BLUE ? Color2 = 2 : Color1 = 2; // Color1 - Color2
    if(BUFF_is_BLUE)
    {
        Color2 = 2;
    }
    else
    {
        Color1 = 2;
        GrayThreshold = Red_Gray_Value + 1;
        ColorThreshold = Red_Value + 1;
    }
    
    //cout << Color1 << "  " << Color2 << endl;
    unsigned char* ImgB = SSE_BGR[0].data;
    unsigned char* ImgG = SSE_BGR[1].data;
    unsigned char* ImgR = SSE_BGR[2].data;
    unsigned char* ImgGray = GrayImg.data;
    unsigned char* ImgColor1 = SSE_BGR[Color1].data;
    unsigned char* ImgColor2 = SSE_BGR[Color2].data;
    unsigned char* Dst = OutImg.data;
#pragma omp parallel for
    for (int Y = 0; Y < Height; Y++)
    {
        // cout << omp_get_thread_num() << endl;

        unsigned char* LinePB = ImgB + Y * Width;
        unsigned char* LinePG = ImgG + Y * Width;
        unsigned char* LinePR = ImgR + Y * Width;
        unsigned char* LinePColor1 = ImgColor1 + Y * Width;
        unsigned char* LinePColor2 = ImgColor2 + Y * Width;
        unsigned char* LinePGray = ImgGray + Y * Width;
        unsigned char* LinePD = Dst + Y * Width;
        //unsigned char* LinePS = InImg.ptr<uchar>(Y);
        //unsigned char* LinePD = OutImg.ptr<uchar>(Y);
        for (register int X = 0; X < Block * BlockSize; X += BlockSize)
        {
            //register int PixelLocaion = X * 3;
            __m128i SrcColor1, SrcColor2 , SrcGray;
            SrcColor1 = _mm_loadu_si128((__m128i*)(LinePColor1+X));
            SrcColor2 = _mm_loadu_si128((__m128i*)(LinePColor2+X));
            for (register int i = 0; i < BlockSize; i++)
            {
                *(LinePGray +X+i) = (((float)*(LinePR + X + i) * 0.299 + (float)*(LinePG + X + i) * 0.587 +
                    (float)*(LinePB + X + i) * 0.114)>= GrayThreshold ? 255:0);
            }
            SrcGray = _mm_loadu_si128((__m128i*)(LinePGray+X));
            
            __m128i Src1, Result;
            //Src1 = _mm_loadu_si128((__m128i*)(LinePS + X));
            Src1 = _mm_subs_epu8(SrcColor1, SrcColor2);
            Result = _mm_cmpeq_epi8(_mm_max_epu8(Src1, _mm_set1_epi8(ColorThreshold)), Src1);
            
            _mm_storeu_si128((__m128i*)(LinePD + X), _mm_and_si128(Result, SrcGray));
            //_mm_storeu_si128((__m128i*)(LinePD + X),Result);

            //m1 = _mm_mul_ps(*pSrc1, *pSrc1); // m1 = *pSrc1 * *pSrc1
            //m2 = _mm_mul_ps(*pSrc2, *pSrc2); // m2 = *pSrc2 * *pSrc2
            //m3 = _mm_add_ps(m1, m2); // m3 = m1 + m2
            //m4 = _mm_sqrt_ps(m3); // m4 = sqrt(m3)
            //*pDest = _mm_add_ps(m4, m0_5); // *pDest = m4 + 0.5			
        }

        for (register int X = Block * BlockSize; X < Width; X++)
        {
            uchar GrayValue = ((float)*(LinePR + X) * 0.299 + (float)*(LinePG + X) * 0.587 + (float)*(LinePB + X) * 0.114) >= GrayThreshold ? 255 : 0;
            //LinePD[X] = LinePColor1[X * 3 + Color1] - LinePColor2[X * 3 + Color2] >= nThreshold ? 255 : 0;
            LinePD[X] = (LinePColor1[X + Color1] - LinePColor2[X + Color2] >= ColorThreshold ? 255 : 0) & GrayValue;
        }
    }
    // Mat element = getStructuringElement(MORPH_RECT, Size(3,3));//
    // dilate(OutImg, OutImg, element);//白大

    return OutImg;
}