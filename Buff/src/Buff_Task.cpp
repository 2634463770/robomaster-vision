#include "Buff_Task.h"
// int Detect_Mode = 0;//1
// int shootmode = 1;//small big 静止
 atomic<int> shootmode;
int speedmode = 0;//30射速 18射速
double Buff_R = 835.0;//能量机关世界坐标半径
int fire_flag = 100;//开火标志
bool Curr_fire_flag = 100;//当前开火标志
int fire_confidence = 0;//开火置信度
int change_confidence = 0;//扇叶改变的置信度
double add_predict_time = 0.0;//预测时间 补偿
double add_t0_time = 0.0;//t0起始 补偿
double buff_pitch_up_down = 0.0;//pitch补偿
double buff_yaw_left_right = 0.0;//yaw补偿
// static VIDEO(SERIAL_PATH);
int V_direction = 0;//1=ringht -1=left 顺时针1 逆时针-1 未知0
int is_activated = 0;//已经激活次数
int is_disappear = 0;//已经消失次数
extern mutex mtx;

/*     (w)
 *  0--------1              90度
 *  |        | (h)        /     \
 *  3--------2       (+-)180度   0度
 *     |  |               \     /
 *     |  |                -90度
 */



bool Buff_Class::Buff_Task(Mat &src)
{   
    
    struct timeval t_start;
    // struct timeval t1,t2;
    struct timeval t_end;

    if(shootmode==0 && Buff.sin_A>0)
        clear_data();
    Buff.clear_flag = false;
    Buff_init();

    {


        if(Buff.temp.size()<2)
        {
            Buff.temp.push_back(imread(Inaction_Path,0));
            Buff.temp.push_back(imread(Action_Path,0));
        }    
        // if(Buff.temp[0].empty() || Buff.temp[0].empty())
        //     return;
        if (shootmode == 1 && CameraMode !=1)
            CameraMode = 1;
        if(GALAXY_EXPOSURE_TIME !=1500)
            GALAXY_EXPOSURE_TIME = 1500;
        if(src.empty())
        {
            printf("could not load image\n");
            return false;
        }
        extern double add_predict_time;
        extern double add_t0_time;
        extern double Buff_R;
        extern double buff_pitch_up_down;
        extern double buff_yaw_left_right;
        extern double Buff_R;
        // extern Buff_Class buff;
        extern vector<RotatedRect> undetermined_inaction_target;
        // if(undetermined_inaction_target.size()==0)
        //     waitKey(0);
        extern bool UpdateSin;
        switch (keyboard_key)
        {
        case 'r' : Buff_R = Buff_R + 1; break;
        case 't' : Buff_R = Buff_R - 1; break;
        case 'w' : buff_pitch_up_down =  buff_pitch_up_down - 1; break;
        case 's' : buff_pitch_up_down = buff_pitch_up_down + 1; break;
        case 'a' : buff_yaw_left_right = buff_yaw_left_right - 1; break;
        case 'd' : buff_yaw_left_right = buff_yaw_left_right + 1; break;
        case 'g' : add_predict_time = add_predict_time + 1; break;
        case 'h' : add_predict_time = add_predict_time - 1; break;
        case 'j' : add_t0_time = add_t0_time + 1; break;
        case 'k' : add_t0_time = add_t0_time - 1; break;
        case 'b' : UpdateSin = true ; break;
        }



    }

//-----------------------Task Start---------------------//


    gettimeofday(&t_start,NULL); // 获取时钟计数
    Mat light_color = Pretreat_img(src);
    // Mat light_color = ForImg(src);
    // Mat light_color = SSEImg(src);


    bool Buff_Found = Find_buff(Buff.inaction, light_color,src);

    light_color.release();

    if (Buff_Found)
    {
        if(Buff.first_find == false)
            Buff.first_find=true;//第一次识别 
#ifdef DEBUG_PIC
        if(ui_open)
        { 
            Point2f inaction_pts[4];
            Buff.inaction.points(inaction_pts);
            for (int i = 0;i<4;i++) 
            {
                line(src,inaction_pts[i],inaction_pts[(i+1)%4],Scalar(0,255,0),2);
            }
            for (int i =0;i<Buff.action.size();i++) 
            {
                Point2f action_pts[4];
                Buff.action[i].points(action_pts);

                for (int j = 0;j<4;j++) 
                {
                    line(src,action_pts[j],action_pts[(j+1)%4],Scalar(255,120,255),2);            
                }

            }
            double max_length = max(Buff.inaction.size.width,Buff.inaction.size.height);
            rectangle(src,Point(Buff.Last_pts.x-max_length,Buff.Last_pts.y-max_length),Point(Buff.Last_pts.x+max_length,Buff.Last_pts.y+max_length),Scalar(100,100,100));
        }
#endif
        update_Vdata();//更新数据


        {
            // 这些是4、5片连打模式，不成熟，不换摄像头测试不了，看不懂可以不管
            // if(Buff.action.size() < 2 && shootmode != 1)
            // {
            //     Buff.pnp_time = getCurrentTime();
            // }
            // if(double(getCurrentTime()-Buff.pnp_time)/1000.0 < 1.0 && Buff.atk_final && shootmode != 1 && atk_mode){
            //     // Add_Pnp_buff(Point2f(),false,src);
            //     Add_Pnp_buff(Point2f(),false,src);
            //     cout<<"atk the 4th inaction"<<endl;
            // }
            // else if (double(getCurrentTime()-Buff.pnp_time)/1000.0 < 4.0 && Buff.atk_final && Buff.action.size() == 3 && shootmode != 1 && atk_mode){//够4秒则转移位置
            //     // Add_Pnp_buff(Buff.final_inaction,true,src);
            //     Add_Pnp_buff(Buff.final_inaction,true,src);
            //     cout<<"atk final inaction"<<endl;
            // }else 
            // {
                Add_Pnp_buff(Point2f(),false,src);
            // }
        }

        Buff.Last_angle = 360.0 * Buff.rotations + Buff.angle;
        Buff.Last_pts = Buff.inaction.center;
        is_activated = 0;
        is_disappear = 0;
    }
    else 
    {
#ifdef DEBUG_LOG
      cout<<"can't not find buff"<<endl;
#endif

        MissBuff(is_disappear,is_activated);

    }
//-----------------------Task End---------------------//

    Buff.action.clear();
    Buff.action.shrink_to_fit();
    Buff.action_fan.clear();
    Buff.action_fan.shrink_to_fit();
    Buff.pts.clear();
    Buff.pts.shrink_to_fit();

    gettimeofday(&t_end,NULL); // 获取时钟计数

    float time_use=((t_end.tv_sec-t_start.tv_sec)*1000000+(t_end.tv_usec-t_start.tv_usec))/1000.0;//hao秒 ms


    return true;
}

