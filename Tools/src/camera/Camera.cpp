/***
 *
 *
 *                                                    __----~~~~~~~~~~~------___
 *                                   .  .   ~~//====......          __--~ ~~
 *                   -.            \_|//     |||\\  ~~~~~~::::... /~
 *                ___-==_       _-~o~  \/    |||  \\            _/~~-
 *        __---~~~.==~||\=_    -_--~/_-~|-   |\\   \\        _/~
 *    _-~~     .=~    |  \\-_    '-~7  /-   /  ||    \      /
 *  .~       .~       |   \\ -_    /  /-   /   ||      \   /
 * /  ____  /         |     \\ ~-_/  /|- _/   .||       \ /
 * |~~    ~~|--~~~~--_ \     ~==-/   | \~--===~~        .\
 *          '         ~-|      /|    |-~\~~       __--~~
 *                      |-~~-_/ |    |   ~\_   _-~            /\
 *                           /  \     \__   \/~                \__
 *                       _--~ _/ | .-~~____--~-/                  ~~==.
 *                      ((->/~   '.|||' -_|    ~~-/ ,              . _||
 *                                 -_     ~\      ~~---l__i__i__i--~~_/
 *                                 _-~-__   ~)  \--______________--~~
 *                               //.-~~~-~_--~- |-------~~~~~~~~
 *                                      //.-~~~--\
 *                                 神兽保佑
 *                                代码无BUG!
 */

#include "Camera.h"
#include <fcntl.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "Base.h"
#include "Buff_Task.h"

atomic<int> CameraMode;//1 是外触发
atomic<double> GALAXY_EXPOSURE_TIME;//曝光时间
Mat ExternalImage;//外触发时用的图像
char* External_rgb_image = nullptr;
mutex Img_mtx;
atomic<bool> External_Image_Finish;
extern Buff_Class buff;

CameraDevice::CameraDevice()
{
    status = GX_STATUS_SUCCESS;
    //    GX_DEV_HANDLE hDevice = nullptr;
    //    uint32_t nDeviceNum = 0;
    // 工业相机尺寸//1920*1080//640*360

#ifdef SHORT
    src.create(480, 640, CV_8UC3);
#else
    src.create(960, 1280, CV_8UC3); // 工业相机尺寸
#endif

    stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE;
    stOpenParam.openMode = GX_OPEN_INDEX;
    stOpenParam.pszContent = "1";
    nFrameNum = 0;
}

CameraDevice::~CameraDevice()
{
    status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_STOP);

    //释放图像缓冲区buffer
    free(stFrameData.pImgBuf);

    status = GXCloseDevice(hDevice);
    status = GXCloseLib();
}

void CameraDevice::CameraModeSwitch()
{
    if (Curr_EXPOSURE_TIME != GALAXY_EXPOSURE_TIME)
    {
        status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, GALAXY_EXPOSURE_TIME);
        Curr_EXPOSURE_TIME = 0;
        double nVal = 0;
        if (GXGetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, &nVal) == 0)
            Curr_EXPOSURE_TIME = nVal;
        // cout<<Curr_EXPOSURE_TIME<<endl;
    }

    if (CameraCurrMode != CameraMode)
    {

        status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_STOP);
        if (status != 0)
            return;
        // sleep(1);
        // usleep(10000);
        if (CameraMode)
        {
            //                设置触发模式为 ON
            GXSetEnum(hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
            //设置触发激活方式为上升沿
            status = GXSetEnum(hDevice, GX_ENUM_TRIGGER_ACTIVATION, GX_TRIGGER_ACTIVATION_RISINGEDGE);
            status = GXSetEnum(hDevice, GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_LINE2);
            status = GXSetEnum(hDevice, GX_ENUM_LINE_SELECTOR, GX_ENUM_LINE_SELECTOR_LINE2);
            //                注册图像处理回调函数
            GXRegisterCaptureCallback(hDevice, NULL, OnFrameCallbackFun);

            GXDeviceOfflineCallBack(NULL);
        }
        else
        {
            //注销采集回调
            status = GXUnregisterCaptureCallback(hDevice);
            //设置触发模式为 OFF
            status = GXSetEnum(hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
        }
        int64_t nVal = 0;
        GX_STATUS status = GXGetEnum(hDevice, GX_ENUM_TRIGGER_MODE, &nVal);
        if (status == 0)
            CameraCurrMode = nVal;
        if (CameraCurrMode == CameraMode)
            cout << "------ Success!!! CameraMode: " << CameraMode << " ------" << endl;
        //发送开始采集命令
        status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_START);
    }
}

static void GX_STDC OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM *pFrame) //外触发 回调函数
{
    if (pFrame->status == 0)
    {
        //对图像进行某些操作
        //图像获取成功
        struct timeval t_start;
        gettimeofday(&t_start,NULL); // 获取时钟计数
        // char* m_rgb_image = nullptr; //增加的内容
        // m_rgb_image=new char[pFrame->nWidth*pFrame->nHeight*3];
        if(External_rgb_image==nullptr)
            External_rgb_image = new char[pFrame->nWidth*pFrame->nHeight*3];
        // cout<<"ID"<<pFrame->nFrameID<<endl;
        void* pChar = const_cast<void*>(pFrame->pImgBuf);
        DxRaw8toRGB24(pChar,External_rgb_image,pFrame->nWidth, pFrame->nHeight,RAW2RGB_NEIGHBOUR3,DX_PIXEL_COLOR_FILTER(BAYERBG),false);
        Img_mtx.lock();
        buff.Buff.ThisFrameTime=t_start;
        External_Image_Finish = true;
        if(ExternalImage.empty())
            ExternalImage.create(480,640,CV_8UC3);
        memcpy(ExternalImage.data,External_rgb_image,pFrame->nWidth*pFrame->nHeight*3);
        Img_mtx.unlock();
        //对图像进行处理...
        // delete []m_rgb_image;
    }
    return;
}

int CameraDevice::init()
{
    // 初始化库
    status = GXInitLib();

    if (status != GX_STATUS_SUCCESS)
    {
        return 0;
    }
    status = GXUpdateDeviceList(&nDeviceNum, 1000);

    if ((status != GX_STATUS_SUCCESS) || (nDeviceNum <= 0))
    {
        return 0;
    }
    status = GXOpenDevice(&stOpenParam, &hDevice);
    std::cout << "GxStatus: " << status << std::endl;
    if (status == GX_STATUS_SUCCESS)
    {
        int64_t nPayLoadSize = 0;
        //获取图像buffer大小，下面动态申请内存
        status = GXGetInt(hDevice, GX_INT_PAYLOAD_SIZE, &nPayLoadSize);

        if (status == GX_STATUS_SUCCESS && nPayLoadSize > 0)
        {
            //定义GXGetImage的传入参数
            // GX_FLOAT_RANGE shutterRange;

            // double dExposureValue = 200.0;

            // status = GXGetFloatRange(hDevice, GX_FLOAT_EXPOSURE_TIME, &shutterRange);

            //根据获取的图像buffer大小m_nPayLoadSize申请buffer
            stFrameData.pImgBuf = malloc((size_t)nPayLoadSize);

            // 设置曝光值
            // status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_DELAY, dExposureValue);
            // status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, shutterRange.dMin);
            // status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, shutterRange.dMax);
            status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, GALAXY_EXPOSURE_TIME);
            // status = GXSetFloat(hDevice, GX_ENUM_EXPOSURE_AUTO, GALAXY_EXPOSURE_TIME);
            if (status == 0)
                Curr_EXPOSURE_TIME = GALAXY_EXPOSURE_TIME;

            //设置采集模式连续采集
            //                        status = GXSetEnum(hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
            status = GXSetEnum(hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
            //                        status = GXSetInt(hDevice, GX_INT_ACQUISITION_SPEED_LEVEL, 1);
            // status = GXSetEnum(hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);

            //设置ROI
            //    int64_t nWidth   = 640;
            //    int64_t nHeight  = 480;
            //    int64_t nOffsetX = 0;
            //    int64_t nOffsetY = 0;
            //    status = GXSetInt(hDevice, GX_INT_WIDTH, nWidth);
            //    status = GXSetInt(hDevice, GX_INT_HEIGHT, nHeight);
            //    status = GXSetInt(hDevice, GX_INT_OFFSET_X, nOffsetX);
            //    status = GXSetInt(hDevice, GX_INT_OFFSET_Y, nOffsetY);

            if (CameraMode)
            {
                //设置触发模式为 ON
                status = GXSetEnum(hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
                //设置触发激活方式为上升沿
                status = GXSetEnum(hDevice, GX_ENUM_TRIGGER_ACTIVATION, GX_TRIGGER_ACTIVATION_RISINGEDGE);
                status = GXSetEnum(hDevice, GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_LINE2);
                status = GXSetEnum(hDevice, GX_ENUM_LINE_SELECTOR, GX_ENUM_LINE_SELECTOR_LINE2);
                //注册图像处理回调函数
                status = GXRegisterCaptureCallback(hDevice, NULL, OnFrameCallbackFun);
            }
            else
            {
                //设置触发模式为 OFF
                status = GXSetEnum(hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
                //注销采集回调
                status = GXUnregisterCaptureCallback(hDevice);
            }
            cout << "------ Success!!! CameraMode: " << CameraMode << " ------" << endl;

            int64_t nVal = 0;
            status = GXGetEnum(hDevice, GX_ENUM_TRIGGER_MODE, &nVal);
            if (status == 0)
                CameraCurrMode = nVal;
            //发送开始采集命令
            status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_START);
            return 1;
        }
    }
    return 0;
}

void CameraDevice::getImage(Mat &img)
{
    GXFlushQueue(hDevice);
    GXGetImage(hDevice, &stFrameData, 100);
    // usleep(1);

    if (stFrameData.nStatus == GX_FRAME_STATUS_SUCCESS)
    {
        //图像获取成功
        char *m_rgb_image = nullptr; //增加的内容
        m_rgb_image = new char[stFrameData.nWidth * stFrameData.nHeight * 3];
        DxRaw8toRGB24(stFrameData.pImgBuf, m_rgb_image, stFrameData.nWidth, stFrameData.nHeight, RAW2RGB_NEIGHBOUR3, DX_PIXEL_COLOR_FILTER(BAYERBG), false);

        memcpy(src.data, m_rgb_image, stFrameData.nWidth * stFrameData.nHeight * 3);

        src.copyTo(img);
        //        img = src;
        nFrameNum++;

        //对图像进行处理...
        delete[] m_rgb_image;
    }
}

uint64_t CameraDevice::getFrameNumber()
{
    return nFrameNum;
}
