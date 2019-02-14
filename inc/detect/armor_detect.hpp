#ifndef _ARMOR_DETECT_H_
#define _ARMOR_DETECT_H_

#include "utility/xml_loader.hpp"
#include "detect/light_detect.hpp"

namespace Hero_Vision
{
namespace vision_mul
{

struct Armor_info {
    struct Armor_points {
        cv::Point2f left_top;
        cv::Point2f left_bottom;
        cv::Point2f right_top;
        cv::Point2f right_bottom;
    } armor_points;
    cv::RotatedRect armor_RRect;
};

class Armor_Detector : public Light_Detector {
public:
    Armor_Detector() = default;
    Armor_Detector(const std::string &params_path): Light_Detector(params_path) { }

    void get_armors(std::vector<cv::RotatedRect> &lights, std::vector<Armor_info> &armors);
    void filter_armors(void);
};

}
}

#endif