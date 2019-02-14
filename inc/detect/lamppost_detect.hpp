#ifndef _LAMPPOST_DETECT_H_
#define _LAMPPOST_DETECT_H_

#include <opencv2/opencv.hpp>
#include "detect/light_detect.hpp"
#include "detect/armor_detect.hpp"

namespace Hero_Vision
{
namespace vision_mul
{

struct lamppost_info {
    cv::RotatedRect lamppost_rect;
    double HP;
};

class Lamppost_Detector: public Light_Detector {
public:
    Lamppost_Detector() = default;
    Lamppost_Detector(const std::string &params_path): Light_Detector(params_path) { }

    void get_lamppost(cv::Mat &frame, std::vector<Armor_info> armors, std::vector<lamppost_info> lampposts);

    void detect_lights(cv::Mat &frame, std::vector<cv::RotatedRect> &lights) const override;
};

}
}

#endif