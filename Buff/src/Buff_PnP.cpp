#include "Buff_Task.h"
bool atk_mode = false;
extern int Detect_Mode;//1
extern  atomic<int> shootmode;
extern int speedmode;///30射速 18射速
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
extern vector<RotatedRect> undetermined_inaction_target;
extern int V_direction;//1=ringht -1=left 顺时针1 逆时针-1 未知0
extern mutex mtx;
inline void CodeRotateByY(double x, double z, double thetay, double& outx, double& outz);
inline void CodeRotateByX(double y, double z, double thetax, double& outy, double& outz);
inline void CodeRotateByZ(double x, double y, double thetaz, double& outx, double& outy);
double h2 = 0; //像素 能量机关高度上的重力补偿
#ifdef Pixel_PNP
struct PnPData
{
    double z;//距离
    double p;//pitch
    double y;//yaw
};
vector<PnPData> pnp_data; //保存pnp数据
FILE* PnpDataFile;

#endif //Pixel_PNP

void Buff_Class::Add_Pnp_buff(Point2f pre_center, bool deviate,Mat &output)//pnp
{
    /*------------------------------pnp    距离------------------------------*/
    Mat object;
    Mat cameraMatrix, distCoeffs;
    vector<Point3f> objectPoints;
    vector<Point2f> project;
    vector<Point2f> imagePoints;
    Mat rvec(3, 1, DataType<double>::type);
    Mat tvec(3, 1, DataType<double>::type);
    double Dist, pitch, yaw;//Dist摄像头到装甲板中心的距离
    double predict_angle = 0.0;
    extern double add_predict_time;
    extern double add_t0_time;
    extern double Buff_R;

    //补偿改变的间隔
#ifdef Pixel_PNP
    const double PitchChange = 5.0,Yawchange = 5.0;
#else
    const double PitchChange = 0.001,Yawchange = 0.005;
#endif   

#ifndef DEBUG_MODE
    //保存3个相同的 串口上的补偿数据 预测时间 pitch yaw 
    //并且可以通过串口调补偿
    //看不懂可以不管
    //已经注释这功能了
    // if
    // (
    //     buff_pitch_up_down != -1.0*ds.PitchCompensation ||
    //     buff_yaw_left_right != -1.0*ds.PredictionCompensation ||
    //     add_predict_time != ds.PredictionCompensation 
    // )
    // {
    //     buff_pitch_up_down = -1.0*ds.PitchCompensation;
    //     buff_yaw_left_right = -1.0*ds.YawCompensation;
    //     add_predict_time = ds.PredictionCompensation;

    //     extern string FileData_result;
    //     string File_Path_1;
    //     string File_Path_2;
    //     string File_Path_3;
    //     if(access("../DataLog/", 0) == -1)
    //     {
    //         mkdir("../DataLog/",S_IRWXU);
    //     }
    //     File_Path_1= "../DataLog/"+std::string(FileData_result)+"_1.txt";
    //     File_Path_2= "../DataLog/"+std::string(FileData_result)+"_2.txt";
    //     File_Path_3= "../DataLog/"+std::string(FileData_result)+"_3.txt";
    //     FILE *DataLogFile1=NULL;
    //     FILE *DataLogFile2=NULL;
    //     FILE *DataLogFile3;
    //     if(DataLogFile1==NULL)
    //     {
    //         DataLogFile1=fopen(File_Path_1.c_str(),"w+");
    //         fprintf(DataLogFile1, "%f %f %f\n", add_predict_time,buff_pitch_up_down,buff_yaw_left_right);
    //         fclose(DataLogFile1);
    //         DataLogFile2=fopen(File_Path_2.c_str(),"w+");
    //         fprintf(DataLogFile2, "%f %f %f\n", add_predict_time,buff_pitch_up_down,buff_yaw_left_right);
    //         fclose(DataLogFile2);
    //         DataLogFile3=fopen(File_Path_3.c_str(),"w+");
    //         fprintf(DataLogFile3, "%f %f %f\n", add_predict_time,buff_pitch_up_down,buff_yaw_left_right);
    //         fclose(DataLogFile3);
    //     }
    // }else
    // {
    //     buff_pitch_up_down = -1.0*ds.PitchCompensation;
    //     buff_yaw_left_right = -1.0*ds.YawCompensation;
    //     add_predict_time= ds.PredictionCompensation;
    // }
#endif

    double Buff_Pitch_Compensation = buff_pitch_up_down*PitchChange;
    double Buff_Yaw_Compensation = buff_yaw_left_right*Yawchange;
    double Buff_Predict_Compensation = add_predict_time*0.01;
    double Buff_t0_Compensation = add_t0_time*0.001;

    if(shootmode != 1)
    {
        int buff_x=0,buff_y=0;
        if(shootmode==0){//小能量机关
            struct timeval CurrTime;
            gettimeofday(&CurrTime,NULL); // 获取时钟计数
            //这五行是计算小能量机关获得角度 的方法
            // double CodeUseTime = (((CurrTime.tv_sec-Buff.ThisFrameTime.tv_sec)*1000000+(CurrTime.tv_usec-Buff.ThisFrameTime.tv_usec))/1000.0)/1000;
            // double predict_time = 6.439/(ds.bullet_velocity *cos(-1.0*ds.camera_pit_angle/180.0*PI))+CodeUseTime+Buff_Predict_Compensation+PredictTime_Compensation-0.016-0.029;//+0.065;
            // double predict_angle = (10.0 * 2.0 * PI / 60.0) * predict_time; //数据  看规则
            // buff_x = 1.0* Buff_R * sin(predict_angle);
            // buff_y = 1.0* Buff_R *(1.0-cos(predict_angle));

            //定死不变的预测量的方法
            buff_x=265;
            buff_y=48;

        }

        if(V_direction==1)
            buff_x = -abs(buff_x);
        else if (V_direction==-1)
            buff_x=+abs(buff_x);
        objectPoints.clear();
        objectPoints.push_back(Point3f(-145 + buff_x, -105 - buff_y, 0));//x+250 y-50
        objectPoints.push_back(Point3f( 145 + buff_x, -105 - buff_y, 0));
        objectPoints.push_back(Point3f( 145 + buff_x,  105 - buff_y, 0));
        objectPoints.push_back(Point3f(-145 + buff_x,  105 - buff_y, 0));
    }
    else if(shootmode == 1)//打能量机关
    {
        struct timeval CurrTime;
        gettimeofday(&CurrTime,NULL); // 获取时钟计数
        double CodeUseTime = (((CurrTime.tv_sec-Buff.ThisFrameTime.tv_sec)*1000000.0+(CurrTime.tv_usec-Buff.ThisFrameTime.tv_usec))/1000.0)/1000.0;//这帧代码使用的时间
        double predict_time =0;
        if(speedmode==1)
        {
            // predict_time = 6.661/18.0+Buff_Predict_Compensation;
        }
        else //30射速
        {
            //积分  6.439是打符点到能量机关的大概距离
            predict_time = 6.439/(ds.bullet_velocity *cos(-1.0*ds.camera_pit_angle/180.0*PI))+CodeUseTime+Buff_Predict_Compensation+PredictTime_Compensation;//+0.065;

        }
#ifdef DEBUG_LOG
        cout << "Buff_R:" << Buff_R << "  Buff_Predict_Compensation:" << Buff_Predict_Compensation <<"  Buff_t0_Compensation" << Buff_t0_Compensation <<endl;
#endif
        mtx.lock();
        double t0 = (((CurrTime.tv_sec-Buff.tt1.tv_sec)*1000000.0+(CurrTime.tv_usec-Buff.tt1.tv_usec))/1000.0)/1000.0 + add_t0_time + 0.039;
        mtx.unlock();
        //积分 原本积分后是-1.0，  -1.1是我自己改的 
        predict_angle = -1.1*Buff.sin_A/2.0/PI/Buff.sin_f*(cos(2.0*PI *Buff.sin_f * (predict_time+t0)+Buff.sin_theta) - cos(2.0*PI*Buff.sin_f*t0+Buff.sin_theta))
            + Buff.sin_C*predict_time;


#ifdef DEBUG_PIC
        if(ui_open)
        { 
            circle(output,calcPoint(Buff.center,Buff.radius,Buff.angle/180.0*PI-predict_angle),3,Scalar(255,255,0),-1);
        }
#endif
        int x=0;
        int y=0;
        if(Buff.sin_A>0)
        {
            // x = Buff_R * cos(predict_angle-PI / 2.0);
            // y = Buff_R * sin(predict_angle-PI / 2.0)+Buff_R;

            // x = -1.0* Buff_R * cos(predict_angle-3.1416 / 2.0);
            // y = Buff_R * sin(predict_angle-3.1416 / 2.0)+Buff_R;

            x = 1.0* Buff_R * sin(predict_angle);
            y = 1.0* Buff_R *(1.0-cos(predict_angle));//正弦定理
        }
        else 
        {
            x = 0;
            y = 0;
        }
        if(V_direction==1)
            x=-abs(x);
        else if(V_direction==-1)
            x=abs(x);
        objectPoints.clear();
        objectPoints.push_back(Point3f(-145+x, -105-y, 0));//x+250 y-50
        objectPoints.push_back(Point3f( 145+x, -105-y, 0));
        objectPoints.push_back(Point3f( 145+x,  105-y, 0));
        objectPoints.push_back(Point3f(-145+x,  105-y, 0));
    }
    Mat(objectPoints).convertTo(object, CV_32F);
    if (deviate) //根据偏移到未出现第五片扇叶，为了4、5连打而写的，看不懂可以不管
    {
        Point2f pre_pts[4];
        float dx = pre_center.x - Buff.inaction.center.x;
        float dy = pre_center.y - Buff.inaction.center.y;
        pre_pts[0] = Point2f(Buff.pts[0].x + dx, Buff.pts[0].y + dy);
        pre_pts[1] = Point2f(Buff.pts[1].x + dx, Buff.pts[1].y + dy);
        pre_pts[2] = Point2f(Buff.pts[2].x + dx, Buff.pts[2].y + dy);
        pre_pts[3] = Point2f(Buff.pts[3].x + dx, Buff.pts[3].y + dy);
        imagePoints.push_back(pre_pts[0]);
        imagePoints.push_back(pre_pts[1]);
        imagePoints.push_back(pre_pts[2]);
        imagePoints.push_back(pre_pts[3]);
    }
    else
    {
        imagePoints.clear();
#ifdef Pixel_PNP      
        //在像素坐标上的重力补偿
        imagePoints.push_back(Point2f(Buff.pts[0].x+Buff_Yaw_Compensation+Yaw_Compensation,Buff.pts[0].y+Buff_Pitch_Compensation+Pitch_Compensation+h2));
        imagePoints.push_back(Point2f(Buff.pts[1].x+Buff_Yaw_Compensation+Yaw_Compensation,Buff.pts[1].y+Buff_Pitch_Compensation+Pitch_Compensation+h2));
        imagePoints.push_back(Point2f(Buff.pts[2].x+Buff_Yaw_Compensation+Yaw_Compensation,Buff.pts[2].y+Buff_Pitch_Compensation+Pitch_Compensation+h2));
        imagePoints.push_back(Point2f(Buff.pts[3].x+Buff_Yaw_Compensation+Yaw_Compensation,Buff.pts[3].y+Buff_Pitch_Compensation+Pitch_Compensation+h2));
#else
        imagePoints.push_back(Point2f(Buff.pts[0].x,Buff.pts[0].y));
        imagePoints.push_back(Point2f(Buff.pts[1].x,Buff.pts[1].y));
        imagePoints.push_back(Point2f(Buff.pts[2].x,Buff.pts[2].y));
        imagePoints.push_back(Point2f(Buff.pts[3].x,Buff.pts[3].y));
#endif
    }
    Mat(imagePoints).convertTo(project, CV_32F);
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 0) = 1313.556856647713;
    cameraMatrix.at<double>(0, 2) = 302.412469653367;
    cameraMatrix.at<double>(1, 1) = 1313.924431868701;
    cameraMatrix.at<double>(1, 2) = 253.392308733315;
    cameraMatrix.at<double>(2, 2) = 1;
    distCoeffs = Mat::zeros(5, 1, CV_64F);
    distCoeffs.at<double>(0, 0) = -0.1903;
    distCoeffs.at<double>(1, 0) = -0.1527;
    distCoeffs.at<double>(2, 0) = 0;
    distCoeffs.at<double>(3, 0) = 0;
    distCoeffs.at<double>(4, 0) = 0;
    solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
    Mat rvecM(3, 3, cv::DataType<double>::type);
    Rodrigues(rvec, rvecM);
    double thetaZ = atan2(rvecM.at<double>(1, 0), rvecM.at<double>(0, 0)) / PI * 180;
    double thetaY = atan2(-1 * rvecM.at<double>(2, 0), sqrt(rvecM.at<double>(2, 1) * rvecM.at<double>(2, 1)
        + rvecM.at<double>(2, 2) * rvecM.at<double>(2, 2))) / PI * 180;
    double thetaX = atan2(rvecM.at<double>(2, 1), rvecM.at<double>(2, 2)) / PI * 180;
    /*translation matrix*/
    double tx = tvec.ptr<double>(0)[0];
    double ty = tvec.ptr<double>(0)[1];
    double tz = tvec.ptr<double>(0)[2];
    double x = tx, y = ty, z = tz;
    /*Three reverse rotations*/
    CodeRotateByZ(x, y, -1 * thetaZ, x, y);
    CodeRotateByY(x, z, -1 * thetaY, x, z);
    CodeRotateByX(y, z, -1 * thetaX, y, z);
    /*WP*/
    double Cx = tx;//左为正
    double Cy = ty;//上为正
    double Cz = tz;//正后方为正
#ifdef Pixel_PNP

    //matlab拟合出的公式
    double p1 = -1.409e-7;//  (-3.177e-06, 2.031e-06)
    double p2 = -0.003554;//  (-0.03623, 0.03464)
    double p3 = 78.09;//  (-42.14, 197.2)
    h2 = p1*Cz*Cz + p2*Cz + p3;//计算在像素上的重力补偿

    //记录PNP的数据，方便记录数据，在matlab导入，看不懂可以不管，就是下需要手动输入maltab
/*
    if(keyboard_key == 'i')
    {
        //记录数据
        //按距离远近排序
        pnp_data.push_back({Cz,Buff_Pitch_Compensation+Pitch_Compensation+h2,Buff_Yaw_Compensation+Yaw_Compensation});       
        // 冒泡排序
        int cnt = pnp_data.size();
        for (int i = 0; i < cnt; ++i) 
        {
            for (int j = 0; j < cnt - 1 - i; ++j) 
            {
                if (pnp_data[j].z > pnp_data[j + 1].z)  // 从小排到大
                {
                    PnPData temp = pnp_data[j];
                    pnp_data[j] = pnp_data[j + 1];
                    pnp_data[j + 1] = temp;
                }
            }
        }
    }
    else if(keyboard_key == 'p')
    {
        //写入数据到文件
        // if(PnpDataFile==NULL)
        PnpDataFile=fopen("../Buff/PnPData.txt","w+");
        for(int i = 0;i<pnp_data.size();i++)
            fprintf(PnpDataFile, "%f %f %f\n", pnp_data[i].z,pnp_data[i].p,pnp_data[i].y);
        fclose(PnpDataFile);
    }
    cout<<"--------------------------------------------------------------"<<endl;
    for(int i = 0;i<pnp_data.size();i++)
    {
        printf("z:%f p:%f y:%f\n",pnp_data[i].z,pnp_data[i].p,pnp_data[i].y);

    }
    cout<<"=============================================================="<<endl;
*/
#endif
    cout<<"Buff_Pitch_Compensation: "<<Buff_Pitch_Compensation+Pitch_Compensation+h2<<"    Buff_Yaw_Compensation: "<<Buff_Yaw_Compensation+Yaw_Compensation
    << "Buff_Predict:"<< Buff_Predict_Compensation+PredictTime_Compensation<<endl;

#ifdef DEBUG_LOG
    cout<<"buff_pitch_up_down: "<<buff_pitch_up_down<<"    buff_yaw_left_right: "<<buff_yaw_left_right<<endl;
#endif

#ifdef Pixel_PNP
    pitch = atan(Cy / Cz) * 180.0 / PI;
    yaw = atan(Cx / Cz) / PI * 180;//绕Y轴旋转
#else
    double tan_alpha = tan(Cy / Cz + Buff_Pitch_Compensation+Pitch_Compensation);
    pitch = (asin(((GRAVITY*Cy/1000.0/ds.bullet_velocity/ds.bullet_velocity)+tan_alpha)/sqrt(1+tan_alpha*tan_alpha))+atan(tan_alpha))/2.0/PI*180.0;//绕X轴旋转
    // pitch = (asin(((GRAVITY*Cy/1000.0/25.0/25.0)+tan_alpha)/sqrt(1+tan_alpha*tan_alpha))+atan(tan_alpha))/2.0/PI*180.0;//绕X轴旋转
    yaw = atan(Cx / Cz) / PI * 180 + Buff_Yaw_Compensation + Yaw_Compensation;//绕Y轴旋转
#endif
    
    //自动开火
    Data_Get dg;    
    if(((abs(pitch)+abs(yaw)<(5.0*predict_angle) && shootmode ==1) || (abs(pitch)+abs(yaw)<1.5 && shootmode !=1)))//云台角足够小
    {
        fire_confidence++;
        // if(fire_confidence>=5 && change_confidence >=20 && Curr_fire_flag == false )
        
        if(fire_confidence>=10 && Buff.inaction_confidence >=5 && Curr_fire_flag == false && undetermined_inaction_target.size()==1)
        //开火置信度 扇叶置信度 开火标志 只存在一片未激活叶片 
        {
            fire_flag++;
            putText(output,"Fire",Point(100,300),FONT_HERSHEY_SIMPLEX,1,Scalar(0,255,0));
            #ifdef DEBUG_LOG
            cout<<"change_confidence"<< Buff.inaction_confidence<<endl;
            cout<<"fire_confidence"<<fire_confidence<<endl;
            cout<<"Fire"<<endl;
            #endif
            Curr_fire_flag = true;
                if(fire_flag>255)
            fire_flag = 11;

        }
        else if (fire_confidence>=70)//击打同一片叶片的情况下，每70帧开火
        {
            fire_confidence=0;
            Curr_fire_flag = false;
        }
    }

    dg.get_xy_data(static_cast<int32_t>((yaw) * 10000.0), static_cast<int32_t>((pitch) * 10000.0 * 1.0), 1,fire_flag);
    serial_.send_data(dg);

#ifdef DEBUG_PIC
    if(ui_open)
    { 
        //显示数据
        char x_name[20];
        char y_name[20];
        char z_name[20];
        char pitch_name[20];
        char yaw_name[20];
        char Pitch_Compensation_name[20];
        char Yaw_Compensation_name[20];
        char Predict_Compensation_name[20];

        sprintf(pitch_name, "pitch:%.4f", pitch);
        sprintf(yaw_name, "yaw:%.4f", yaw);
        sprintf(x_name, "X:%.2f", Cx);
        sprintf(y_name, "Y:%.2f", Cy);
        sprintf(z_name, "Z:%.2f", Cz);
        sprintf(Pitch_Compensation_name, "Pitch:%.4f", Buff_Pitch_Compensation+Pitch_Compensation+h2);
        sprintf(Yaw_Compensation_name, "Yaw:%.4f", Buff_Yaw_Compensation+Yaw_Compensation);
        sprintf(Predict_Compensation_name, "Predict:%.4f", Buff_Predict_Compensation+PredictTime_Compensation);

        putText(output,x_name,Point(10,30),FONT_HERSHEY_SIMPLEX,0.8,Scalar(0,255,0));
        putText(output,y_name,Point(200,30),FONT_HERSHEY_SIMPLEX,0.8,Scalar(0,255,0));
        putText(output,z_name,Point(390,30),FONT_HERSHEY_SIMPLEX,0.8,Scalar(0,255,0));
        putText(output,pitch_name,Point(10,60),FONT_HERSHEY_SIMPLEX,0.8,Scalar(0,255,0));
        putText(output,yaw_name,Point(200,60),FONT_HERSHEY_SIMPLEX,0.8,Scalar(0,255,0));
        putText(output,Pitch_Compensation_name,Point(10,90),FONT_HERSHEY_SIMPLEX,0.8,Scalar(0,255,0));
        putText(output,Yaw_Compensation_name,Point(200,90),FONT_HERSHEY_SIMPLEX,0.8,Scalar(0,255,0));
        putText(output,Predict_Compensation_name,Point(390,90),FONT_HERSHEY_SIMPLEX,0.8,Scalar(0,255,0));
    }
#endif

    //4、5连打，摄像头原因，没有测试过功能是否稳定，可以不管
    double pnp_now =double(getCurrentTime()-Buff.pnp_time)/1000.0;    
    if(!deviate && abs(pitch)<1.0 &&abs(yaw)<1.0 && Buff.action.size() == 3 && Buff.atk_final == false && pnp_now <4.0 && shootmode != 1 && atk_mode)
    {
        Buff.pnp_time = getCurrentTime();
        Buff.atk_final =true;
    }
    else if(Buff.atk_final && pnp_now>4.0 && shootmode != 1 && atk_mode){
        Buff.atk_final = false;
    }
}


inline void CodeRotateByZ(double x, double y, double thetaz, double& outx, double& outy)
{
    double x1 = x;//将变量拷贝一次，保证&x == &outx这种情况下也能计算正确
    double y1 = y;
    double rz = thetaz * PI / 180;
    outx = cos(rz) * x1 - sin(rz) * y1;
    outy = sin(rz) * x1 + cos(rz) * y1;
}
inline void CodeRotateByY(double x, double z, double thetay, double& outx, double& outz)
{
    double x1 = x;
    double z1 = z;
    double ry = thetay * PI / 180;
    outx = cos(ry) * x1 + sin(ry) * z1;
    outz = cos(ry) * z1 - sin(ry) * x1;
}
inline void CodeRotateByX(double y, double z, double thetax, double& outy, double& outz)
{
    //将空间点绕X轴旋转
    //输入参数 y z为空间点原始y z坐标
    //thetax为空间点绕X轴旋转多少度，角度制，范围在-180到180
    //outy outz为旋转后的结果坐标
    double y1 = y;//将变量拷贝一次，保证&y == &y这种情况下也能计算正确
    double z1 = z;
    double rx = thetax * PI / 180;
    outy = cos(rx) * y1 - sin(rx) * z1;
    outz = cos(rx) * z1 + sin(rx) * y1;
}