#ifndef _ARMOR_DETECT_H_
#define _ARMOR_DETECT_H_

#include "utility/xml_loader.hpp"

namespace Hero_Vision
{
namespace vision_mul
{

enum Enemy_Color {RED, BLUE};

class armor_detector {
public:
    void detect_lights(cv::Mat &frame, std::vector<cv::RotatedRect> &lights);
    void filter_lights(std::vector<cv::RotatedRect> &lights);
    void get_armors(std::vector<cv::RotatedRect> &lights, std::vector<cv::RotatedRect> &armors);
    void filter_armors(void);
    void update(void);
    void init(xml_params_loader &_params);
    void detect(cv::Mat &frame, std::vector<cv::RotatedRect> &armors);
private:
    Enemy_Color enemy_color;

    double color_thresh_val_blue;
    double color_thresh_val_red;
    double brightness_thresh_val;
};

}
}

#endif