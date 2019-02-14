#ifndef _VISION_H_
#define _VISION_H_

#include "detect/armor_detect.hpp"
#include "detect/lamppost_detect.hpp"

namespace Hero_Vision
{
namespace vision_mul
{

void vision_process();
void detect(cv::Mat &frame, Armor_Detector &armor_detector, Lamppost_Detector &lamppost_detector);

}

}
#endif
