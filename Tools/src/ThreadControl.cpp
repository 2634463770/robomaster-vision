/**
 *               ii.                                         ;9ABH,
 *              SA391,                                    .r9GG35&G
 *              &#ii13Gh;                               i3X31i;:,rB1
 *              iMs,:,i5895,                         .5G91:,:;:s1:8A
 *               33::::,,;5G5,                     ,58Si,,:::,sHX;iH1
 *                Sr.,:;rs13BBX35hh11511h5Shhh5S3GAXS:.,,::,,1AG3i,GG
 *                .G51S511sr;;iiiishS8G89Shsrrsh59S;.,,,,,..5A85Si,h8
 *               :SB9s:,............................,,,.,,,SASh53h,1G.
 *            .r18S;..,,,,,,,,,,,,,,,,,,,,,,,,,,,,,....,,.1H315199,rX,
 *          ;S89s,..,,,,,,,,,,,,,,,,,,,,,,,....,,.......,,,;r1ShS8,;Xi
 *        i55s:.........,,,,,,,,,,,,,,,,.,,,......,.....,,....r9&5.:X1
 *       59;.....,.     .,,,,,,,,,,,...        .............,..:1;.:&s
 *      s8,..;53S5S3s.   .,,,,,,,.,..      i15S5h1:.........,,,..,,:99
 *      93.:39s:rSGB@A;  ..,,,,.....    .SG3hhh9G&BGi..,,,,,,,,,,,,.,83
 *      G5.G8  9#@@@@@X. .,,,,,,.....  iA9,.S&B###@@Mr...,,,,,,,,..,.;Xh
 *      Gs.X8 S@@@@@@@B:..,,,,,,,,,,. rA1 ,A@@@@@@@@@H:........,,,,,,.iX:
 *     ;9. ,8A#@@@@@@#5,.,,,,,,,,,... 9A. 8@@@@@@@@@@M;    ....,,,,,,,,S8
 *     X3    iS8XAHH8s.,,,,,,,,,,...,..58hH@@@@@@@@@Hs       ...,,,,,,,:Gs
 *    r8,        ,,,...,,,,,,,,,,.....  ,h8XABMMHX3r.          .,,,,,,,.rX:
 *   :9, .    .:,..,:;;;::,.,,,,,..          .,,.               ..,,,,,,.59
 *  .Si      ,:.i8HBMMMMMB&5,....                    .            .,,,,,.sMr
 *  SS       :: h@@@@@@@@@@#; .                     ...  .         ..,,,,iM5
 *  91  .    ;:.,1&@@@@@@MXs.                            .          .,,:,:&S
 *  hS ....  .:;,,,i3MMS1;..,..... .  .     ...                     ..,:,.99
 *  ,8; ..... .,:,..,8Ms:;,,,...                                     .,::.83
 *   s&: ....  .sS553B@@HX3s;,.    .,;13h.                            .:::&1
 *    SXr  .  ...;s3G99XA&X88Shss11155hi.                             ,;:h&,
 *     iH8:  . ..   ,;iiii;,::,,,,,.                                 .;irHA
 *      ,8X5;   .     .......                                       ,;iihS8Gi
 *         1831,                                                 .,;irrrrrs&@
 *           ;5A8r.                                            .:;iiiiirrss1H
 *             :X@H3s.......                                .,:;iii;iiiiirsrh
 *              r#h:;,...,,.. .,,:;;;;;:::,...              .:;;;;;;iiiirrss1
 *             ,M8 ..,....,.....,,::::::,,...         .     .,;;;iiiiiirss11h
 *             8B;.,,,,,,,.,.....          .           ..   .:;;;;iirrsss111h
 *            i@5,:::,,,,,,,,.... .                   . .:::;;;;;irrrss111111
 *            9Bi,:,,,,......                        ..r91;;;;;iirrsss1ss1111
 */

#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>

#include "Base.h"
#include "ThreadControl.h"
#include "DetectArmor.h"
#include "Camera.h"
#include "SerialPort.h"
#include "Buff_Task.h"

#include "Function.h"
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <string>

double time_all;
double time_once;
bool ui_open;

ROI roi_new;
int keyboard_key;
int TestFrames = 0;
extern Buff_Class buff;
atomic<bool> FrameHasUpdated;
#define BUFFER_SIZE 1
#define END_THREAD       \
    if (end_thread_flag) \
        return;

static volatile bool end_thread_flag = false;   // 设置结束线程标志位，负责结束所有线程。
static volatile unsigned int produce_index;     // 图像生成序号，用于线程之间逻辑 sem_t
static volatile unsigned int consumption_index; // 图像消耗序号
// static volatile unsigned int consumption_index; // 图像消耗序号

static ImageData data[BUFFER_SIZE];

static int camera_watch(1);
static int serial_watch(1);

ThreadControl::ThreadControl()
{
    cout << "THREAD TASK ON !!!" << endl;
}

// 图像生成线程
static int add_(1);
void ThreadControl::ImageProduce()
{
#ifdef CAMERA
    cout << " ------ SHORT CAMERA PRODUCE TASK ON !!! ------ " << endl;
    CameraDevice galaxy;

RETURN_USB:
    VideoCapture capture;

    if (/*ds.car_id == 1 && ds.mode_num == 1*/ 0)
    {
        capture.open(add_);
        if (!capture.isOpened())
        {
            cout << "can not load the USB camera" << endl;
            add_++;
            if (add_ > 5)
            {
                add_ = 0;
            }
            goto RETURN_USB;
        }
    }
    else
    {
        camera_watch = galaxy.init();
    }

    // CaptureVideo capvdo(galaxy);

#endif

#ifdef VIDEO
    cout << " ------ SHORT VIDEO PRODUCE TASK ON !!! ------ " << endl;
    VideoCapture capture;
    capture.open(ADD);

    if (!capture.isOpened())
    {
        cout << "can not load the video" << endl;
    }
#endif

    while (1)
    {
#ifdef CAMERA
        if (/*ds.car_id == 1 && ds.mode_num == 1*/ 0)
        {
            capture.read(data[produce_index % BUFFER_SIZE].img);
        }
        else
        {
            galaxy.CameraModeSwitch();
            if (!CameraMode)
            {
                galaxy.getImage(data[produce_index % BUFFER_SIZE].img);
                while (produce_index - consumption_index >= BUFFER_SIZE && CameraMode == 0)
                {
                    END_THREAD;
                }
                ++produce_index;
            }
        }
#endif

#ifdef VIDEO
        while (produce_index - consumption_index >= BUFFER_SIZE)
            END_THREAD;
        capture.read(data[produce_index % BUFFER_SIZE].img);
        ++produce_index;
#endif
        END_THREAD;
    }
}

Mat SaveImage;
// 图像处理线程
void ThreadControl::ImageProcess()
{
    cout << " ------ IMAGE PROCESS TASK ON !!! ------" << endl;
    Mat image;
    double fps;
    FrameHasUpdated = false;
    double start_(0.0), finish_(0.0);
    double t(0.001);
    ui_open = 0;

    watch_dog = 1;

    int hunger_dog(0);

    vector<double> time_add_vec;
    struct timeval t_start1;
    // struct timeval t1,t2;
    struct timeval t_end1;



    while (1)
    {

#ifdef VIDEO
        gettimeofday(&buff.Buff.ThisFrameTime,NULL); // 获取时钟计数

#endif
#ifdef CAMERA
        if (CameraMode)
        {
            while (!External_Image_Finish);
            Img_mtx.lock();
            ExternalImage.copyTo(image);
            Img_mtx.unlock();
            TestFrames = 1;
            External_Image_Finish = false;            
        }
        else
        {
            // 等待图像生成后进行处理
            while (produce_index - consumption_index == 0);
            data[consumption_index % BUFFER_SIZE].img.copyTo(image);
            ++consumption_index;
        }
#endif

#ifdef VIDEO
        while (produce_index - consumption_index == 0)
        {
        }
        data[consumption_index % BUFFER_SIZE].img.copyTo(image);
        ++consumption_index;
#endif
        gettimeofday(&t_start1,NULL); // 获取时钟计数

        if (image.empty())
        {
            cout << "can not load the picture" << endl;
            end_thread_flag = true;
        }

#ifdef WRITE
        image.copyTo(SaveImage);
        FrameHasUpdated = true;
#endif
        /***********MAIN**************/

#ifdef AREA_DETECT
        image.copyTo(roi_new.input);
#endif

#ifdef PIC
        // imshow("source", image);
#endif

        ModeSwitch(image);

        /***********calculate_time***************/
        //单位：s
        finish_ = (double)getTickCount();
        t = (finish_ - start_) / getTickFrequency();
        start_ = (double)getTickCount();

        time_add_vec.push_back(t);
        time_once = t;

        if (time_add_vec.size() >= ONE_SIZE)
        {
            time_add_vec.erase(time_add_vec.begin());
        }
        double time_sum_ = 0.0;
        for (size_t i = 0; i < time_add_vec.size(); i++)
        {
            time_sum_ = time_sum_ + time_add_vec[i];
        }
        time_all = time_sum_;

        fps = 1.0 / t;

        if (fps < 10 && t < 100)
        {
            watch_dog = 0;
        }

// #ifdef PIC
        putText(image, "FPS: " + to_string(fps), Point(20, 460), FONT_HERSHEY_COMPLEX, 0.8, Scalar(255, 0, 255), 1);
// #endif
#ifndef NotShowImage
        imshow("killer machine", image);
        keyboard_key = waitKey(WAITKEY_VALUE);
#else
        printf("Time: %.3f\n",t);
#endif
        if (keyboard_key == 27)
            end_thread_flag = true;
        gettimeofday(&t_end1,NULL); // 获取时钟计数

#ifdef WATCHDOG
        // cout << camera_watch << "   " << serial_watch << endl;
        serial_watch = 1;
        if (!camera_watch || !serial_watch)
            hunger_dog++;
        if (hunger_dog > FULL_DOG || !watch_dog)
        {
            end_thread_flag = true;
        }
        cout << camera_watch << "   " << serial_watch << "   " << watch_dog << "   " << hunger_dog << endl;
#endif

        if (keyboard_key == 'o')
        {
            ui_open = !ui_open;
        }

        switch (keyboard_key)
        {
        case 'k':
            GALAXY_EXPOSURE_TIME = GALAXY_EXPOSURE_TIME + 10;
            break;
        case 'c':
            CameraMode = !CameraMode;
            break;
        }
    float time_use=((t_end1.tv_sec-t_start1.tv_sec)*1000000+(t_end1.tv_usec-t_start1.tv_usec))/1000.0;//hao秒 ms

    // printf("Frame time_use is %f ms\n",time_use);

        END_THREAD;
    }
    vector<double>().swap(time_add_vec);
}

void ThreadControl::WriteFrame()
{
#ifdef WRITE
    cout << " ------ WRITE FRAME RECEVICE TASK ON !!! ------" << endl;

    static int save(0);
    char vio_[30];
    bool cat(0);

    VideoWriter writer;

    if (access("../videos", 0))
    {
        mkdir("../videos", S_IRWXU);
    }
    else
    {
    PRE:
        sprintf(vio_, "../videos/%d.avi", int(save));
        if (access(vio_, 0))
        {
            writer.open(vio_, CV_FOURCC('M', 'J', 'P', 'G'), 25.0, Size(640, 480));
            cat = 1;
        }
        else
        {
            save++;
            goto PRE;
        }
    }
#endif

    while (1)
    {
        // cout << "wwww" << endl; 
        serial_watch = serial_.read_data();
        // cout<<"Ssss"<<endl;

        #ifdef WRITE

        if (cat && FrameHasUpdated)
        { 
            writer << SaveImage;
            if(CameraMode == 1)
                FrameHasUpdated = false;
        }
        #endif

    }
   
}

extern bool XML_For_Prediction;
double ****SinDataSheet = NULL;
void ThreadControl::PredictProcess()
{
    // buff.MakeDataXML(XML_Path);
    // struct timeval t_start;
    // struct timeval t_end;
    // SinDataSheet = buff.LoadDataXML(XML_Path);
    if(SinDataSheet==NULL)
        XML_For_Prediction = false;
    else
        XML_For_Prediction = true;
    // printf("Buff XML_For_Prediction: %d\n", XML_For_Prediction);

    while (1)
    {
        // if(TestFrames)
        // {
        //     sleep(1);
        //     end_thread_flag = true;
        // }
        // shootmode = ds.DetectionMode-1;
        // shootmode =1;
        if (shootmode == 1)
        {
            // gettimeofday(&t_start,NULL); // 获取时钟计数
            buff.prediction();
            // gettimeofday(&t_end,NULL); // 获取时钟计数
            // float time_use=((t_end.tv_sec-t_start.tv_sec)*1000000+(t_end.tv_usec-t_start.tv_usec))/1000.0;//hao秒 ms
            // printf("time_use is %.3f ms\n",time_use);

        }
        if(end_thread_flag && XML_For_Prediction)
        {
            for (int i = 0;min_f+double(i)*interval_f<max_f;i++)
            {
                for (int j = 0;min_A+double(j)*interval_A<max_A;j++)
                {
                    for (int k = 0;k<Theta_Size;k++)
                    {
                        free(SinDataSheet[i][j][k]);
                    }
                    free(SinDataSheet[i][j]);
                }
                free(SinDataSheet[i]);
            }
            free(SinDataSheet);
        }
        END_THREAD;
    }
}