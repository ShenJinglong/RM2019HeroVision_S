#ifndef _CAMERA_DRIVER_H_
#define _CAMERA_DRIVER_H_

//#include <sys/types.h>  /**/
//#include <sys/stat.h>   /**/
//#include <termios.h>    /*PPSIX终端控制定义*/
//#include <errno.h>      /*错误号定义*/
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <unistd.h>     /*Unix标准函数定义*/
#include <fcntl.h>      /*文件控制定义*/
#include <iostream>
#include <opencv2/opencv.hpp>

namespace Hero_Vision
{
namespace Driver
{

class Camera_Driver {
public:
    Camera_Driver() = default;
    Camera_Driver(cv::VideoCapture &capture, const std::string &params_path) {
        cv::FileStorage fs(params_path, cv::FileStorage::READ);

        if (!fs.isOpened()) {
            std::cerr << "...Can't open [ " << params_path << " ]" << std::endl;
            std::cerr << "...At camera_driver constructor" << std::endl;
            fs.release();
            return ;
        }

        fs["video_or_camera"] >> video_or_camera;
        if (video_or_camera == 0) {
            fs["video_path"] >> video_path;
            capture.open(video_path);
            fs.release();
        } else {
            fs["usb_cam_id"] >> usb_cam_id;
            fs["exposure_time"] >> exposure_time;

            capture.set(CV_CAP_PROP_FPS, 120);
            capture.open(usb_cam_id);
            if (!capture.isOpened()) {
                std::cerr << "...Can't open [ " << "capture" << " ]" << std::endl;
                std::cerr << "...At camera_driver constructor" << std::endl;
                fs.release();
                return ;
            }
            capture.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M','J','P','G'));
            capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
            capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

            int cam_id;
            if ((cam_id = open(usb_cam_id.c_str(), O_RDWR)) == -1) {
                std::cerr << "...Can't open [ " << "camera" " ]" << std::endl;
                std::cerr << "...At camera_driver constructor" << std::endl;
                fs.release();
                return ;
            }
            struct v4l2_control control_s;
            control_s.id = V4L2_CID_EXPOSURE_AUTO;
            control_s.value = V4L2_EXPOSURE_MANUAL;
            ioctl(cam_id, VIDIOC_S_CTRL, &control_s);

            control_s.id = V4L2_CID_EXPOSURE_ABSOLUTE;
            control_s.value = exposure_time;
            ioctl(cam_id, VIDIOC_S_CTRL, &control_s);
            close(cam_id);

            fs.release();
        }
    }
private:
    std::string usb_cam_id;
    std::string video_path;

    int exposure_time;
    int video_or_camera;
};

}
}

#endif