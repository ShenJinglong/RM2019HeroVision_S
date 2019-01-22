#include <opencv2/opencv.hpp>
#include "common/vision.hpp"
#include "detect/armor_detect.hpp"
#include "utility/xml_loader.hpp"

#define XML_PARAMS_PATH "../conf/params.xml"

namespace Hero_Vision
{
namespace vision_mul
{

void vision_process() {
    cv::Mat frame_forward;
    cv::Mat frame_forward_src;
    std::vector<cv::RotatedRect> armors;
    xml_params_loader _params(XML_PARAMS_PATH);
    _params.load();
    cv::VideoCapture frame_capture(_params.video_path);
//    camera_driver hero_camera();
//    hero_camera.init();
//    hero_camera.open();

//    geometry_process hero_geometry();
//    hero_geometry.init();

    armor_detector detector(_params);
    while (1) {
        armors.clear();

        frame_capture >> frame_forward;
        detector.detect(frame_forward, armors);
        cv::imshow("result", frame_forward);
        cv::waitKey(1);
    }
}

}
}