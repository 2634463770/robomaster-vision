/*
//                                                     _ooOoo_
//                                                    o8888888o
//                                                    88" . "88
//                                                    (| -_- |)
//                                                     O\ = /O
//                                                 ____/`---'\____
//                                                  . ' \\| |// `.
//                                                / \\||| : |||// \
//                                              / _||||| -:- |||||- \
//                                                | | \\\ - /// | |
//                                               | \_| ''\---/'' | |
//                                               \ .-\__ `-` ___/-. /
//                                            ___`. .' /--.--\ `. . __
//                                         ."" '< `.___\_<|>_/___.' >'"".
//                                        | | : `- \`.;`\ _ /`;.`/ - ` : | |
//                                          \ \ `-. \_ __\ /__ _/ .-` / /
//                                  ======`-.____`-.___\_____/___.-`____.-'======
//                                                     `=---='
//
//                                    .............................................
//                                           佛祖保佑             永无BUG
//                                   佛曰:
//                                           写字楼里写字间，写字间里程序员；
//                                           程序人员写程序，又拿程序换酒钱。
//                                           酒醒只在网上坐，酒醉还来网下眠；
//                                           酒醉酒醒日复日，网上网下年复年。
//                                           但愿老死电脑间，不愿鞠躬老板前；
//                                           奔驰宝马贵者趣，公交自行程序员。
//                                           别人笑我忒疯癫，我笑自己命太贱；
//                                           不见满街漂亮妹，哪个归得程序员？
*/

#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <string.h>
#include "Camera.h"
#include <ThreadControl.h>
#include "Base.h"

using namespace std;
using namespace cv;
string FileData_result;
int main()
{
    // omp_set_num_threads(8);
    CameraMode = 0; //关闭硬触发
    GALAXY_EXPOSURE_TIME = 1500;
    time_t t = time(NULL);
    char ch[64] = {0};
    char result[100] = {0};
    strftime(ch, sizeof(ch) - 1, "%Y%m%d--%H%M%S", localtime(&t));
    sprintf(result, "%s", ch);
    FileData_result = std::string(result);
    // 开启相关线程
    ThreadControl ImageControl;
    // 图像生成线程
    std::thread produce_task(&ThreadControl::ImageProduce, ImageControl);
    // 图像处理线程
    std::thread process_task(&ThreadControl::ImageProcess, ImageControl);
#ifdef INFANTRY
    // 预测处理线程
    std::thread predict_task(&ThreadControl::PredictProcess, ImageControl);
#endif

#ifdef WRITE
#endif
    std::thread write_task(&ThreadControl::WriteFrame, ImageControl);
    write_task.detach();

    produce_task.join();
    process_task.join();
#ifdef INFANTRY
    predict_task.detach();
#endif

    return 0;
}
