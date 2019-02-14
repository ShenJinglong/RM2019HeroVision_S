#ifndef _LIGHT_DETECT_H_
#define _LIGHT_DETECT_H_

#include <opencv2/opencv.hpp>

namespace Hero_Vision
{
namespace vision_mul
{

enum Enemy_Color {RED, BLUE};

class Light_Detector {
public:
    Light_Detector() = default;
    Light_Detector(const std::string &params_path) {
        cv::FileStorage fs(params_path, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            std::cerr << "...Can't open [ " << params_path << " ]" << std::endl;
            std::cerr << "...At Light_Detector constructor" << std::endl;
            fs.release();
            return ;
        }
        fs["color_thresh_val_blue"] >> color_thresh_val_blue;
        fs["color_thresh_val_red"] >> color_thresh_val_red;
        fs["brightness_thresh_val"] >> brightness_thresh_val;
        
        int temp;
        fs["enemy_color"] >> temp;
        enemy_color = (Enemy_Color)temp;
        fs.release();
    }

    virtual void detect_lights(cv::Mat &frame, std::vector<cv::RotatedRect> &lights) const;
    virtual void filter_lights(std::vector<cv::RotatedRect> &lights);
protected:
    Enemy_Color enemy_color;

    double color_thresh_val_blue;
    double color_thresh_val_red;
    double brightness_thresh_val;
};

}
}

#endif