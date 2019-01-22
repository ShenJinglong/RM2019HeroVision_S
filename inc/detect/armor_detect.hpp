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
    armor_detector(xml_params_loader &_params) {
        enemy_color = (Enemy_Color)_params.enemy_color;

        color_thresh_val_blue = _params.color_thresh_val_blue;
        color_thresh_val_red = _params.color_thresh_val_red;
        brightness_thresh_val = _params.brightness_thresh_val;
        light_weight_ratio = 1;
        light_weight_angle = 1;
        light_weight_area = 1;
    }
    armor_detector() = default;
    void detect_lights(cv::Mat &frame, std::vector<cv::RotatedRect> &lights) const;
    void filter_lights(std::vector<cv::RotatedRect> &lights);
    void get_armors(std::vector<cv::RotatedRect> &lights, std::vector<cv::RotatedRect> &armors);
    void filter_armors(void);
    void update(void);
    void detect(cv::Mat &frame, std::vector<cv::RotatedRect> &armors);
private:
    Enemy_Color enemy_color;

    double color_thresh_val_blue;
    double color_thresh_val_red;
    double brightness_thresh_val;

    double light_weight_ratio;
    double light_weight_angle;
    double light_weight_area;
};

}
}

#endif