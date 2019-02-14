#include <opencv2/opencv.hpp>

#include "common/vision.hpp"

#include "detect/armor_detect.hpp"
#include "detect/lamppost_detect.hpp"
#include "detect/light_detect.hpp"

#include "driver/camera_driver.hpp"

#include "tools/debug_tool.hpp"

#define XML_PARAMS_PATH "../conf/params.xml"
//#define __DEBUG_ARMOR_LIGHT__
//#define __DEBUG_ARMOR_ARMOR__


namespace Hero_Vision
{
namespace vision_mul
{

void vision_process() {
    cv::Mat frame_forward;
//    cv::Mat frame_forward_src;

//    std::vector<cv::RotatedRect> armors;
//    std::vector<cv::RotatedRect> lampposts;
    
    cv::VideoCapture frame_capture;

    Driver::Camera_Driver camera_driver(frame_capture, XML_PARAMS_PATH);
    Armor_Detector armor_detector(XML_PARAMS_PATH);
    Lamppost_Detector lamppost_detector(XML_PARAMS_PATH);

    while (1) {
//        armors.clear();
//        lampposts.clear();

        frame_capture >> frame_forward;

        detect(frame_forward, armor_detector, lamppost_detector);

        cv::imshow("result", frame_forward);
        cv::waitKey(1);
    }
}

void detect(cv::Mat &frame, Armor_Detector &armor_detector, Lamppost_Detector &lamppost_detector) {
    std::vector<cv::RotatedRect> lights;
    std::vector<Armor_info> armors;
    std::vector<lamppost_info> lampposts;

    armor_detector.detect_lights(frame, lights);

#ifdef __DEBUG_ARMOR_LIGHT__
    Tools::show_debug_info("show_lights_after_detect", frame.size(), lights);
#endif

    armor_detector.filter_lights(lights);

#ifdef __DEBUG_ARMOR_LIGHT__
    cv::Mat show_lights_after_filter(frame.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    for (auto &light : lights) {
        Tools::draw_rotated_rect(show_lights_after_filter, light);
    }
    for (int i = 0; i < lights.size(); ++i) {
        cv::putText(show_lights_after_filter, cv::format("%d", i), lights[i].center, CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    }
    imshow("show_lights_after_filter", show_lights_after_filter);
#endif

    armor_detector.get_armors(lights, armors);

#ifdef __DEBUG_ARMOR_ARMOR__
    cv::Mat show_armors_after_get(frame.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    for (auto &armor : armors) {
        Tools::draw_rotated_rect(show_armors_after_get, armor.armor_RRect);
        cv::putText(show_armors_after_get, cv::format("%f", armor.armor_RRect.angle), armor.armor_RRect.center, CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    }
    imshow("show_armors_after_get", show_armors_after_get);
#endif

    lamppost_detector.get_lamppost(frame, armors, lampposts);
}

}
}