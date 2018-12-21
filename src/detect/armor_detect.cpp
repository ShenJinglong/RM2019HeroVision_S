#include <opencv2/opencv.hpp>

#include "detect/armor_detect.hpp"
#include "tools/debug_tool.hpp"
#include "utility/xml_loader.hpp"

#define __DEBUG

namespace Hero_Vision
{
namespace vision_mul
{

void adjust_rect(std::vector<cv::RotatedRect> &RRects) {
    for (auto &RRect : RRects) {
        if (RRect.size.width < RRect.size.height) {
            continue;
        } else {
            double temp;
            temp = RRect.size.height;
            RRect.size.height = RRect.size.width;
            RRect.size.width = temp;
            RRect.angle += 90;
        }
    }
}

void armor_detector::detect(cv::Mat &frame, std::vector<cv::RotatedRect> &armors) {
    std::vector<cv::RotatedRect> lights;

    detect_lights(frame, lights);
    adjust_rect(lights);

#ifdef __DEBUG
    Tools::show_debug_info("show_lights_after_detect", frame.size(), lights);
#endif

    filter_lights(lights);

#ifdef __DEBUG
    cv::Mat show_lights_after_filter(frame.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    for (auto &light : lights) {
        Tools::draw_rotated_rect(show_lights_after_filter, light);
        Tools::draw_rotated_rect_info(show_lights_after_filter, light);
    }
    imshow("show_lights_after_filter", show_lights_after_filter);
#endif

    get_armors(lights, armors);
    filter_armors();
}

void armor_detector::detect_lights(cv::Mat &frame, std::vector<cv::RotatedRect> &lights) {
    cv::Mat color_light;
    std::vector<cv::Mat> bgr_channels;
    cv::split(frame, bgr_channels);

    if (enemy_color == RED) {
        cv::subtract(bgr_channels[2], bgr_channels[1], color_light);
    } else {
        cv::subtract(bgr_channels[0], bgr_channels[1], color_light);
    }

    cv::Mat binary_brightness_img;
    cv::Mat binary_color_img;
    cv::Mat binary_light_img;
    cv::Mat gray_img;

    cv::cvtColor(frame, gray_img, cv::COLOR_BGR2GRAY);
    cv::threshold(gray_img, binary_brightness_img, brightness_thresh_val, 255, CV_THRESH_BINARY);

    cv::threshold(color_light, binary_color_img, enemy_color == RED ? color_thresh_val_red : color_thresh_val_blue, 255, CV_THRESH_BINARY);

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(binary_color_img, binary_color_img, element, cv::Point(-1, -1), 1);

    binary_light_img = binary_color_img & binary_brightness_img;

    std::vector<std::vector<cv::Point> > contours_light;
    cv::findContours(binary_light_img, contours_light, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point> > contours_brightness;
    cv::findContours(binary_brightness_img, contours_brightness, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    lights.reserve(contours_brightness.size());

    std::vector<int> is_processes(contours_brightness.size());

    for (unsigned int i = 0; i < contours_light.size(); ++i) {
        for (unsigned int j = 0; j < contours_brightness.size(); ++j) {
            if (!is_processes[j]) {
                if (cv::pointPolygonTest(contours_brightness[j], contours_light[i][0], true) >= 0.0) {
                    cv::RotatedRect single_light = cv::minAreaRect(contours_brightness[j]);
                    lights.push_back(single_light);
                    is_processes[j] = true;
                    break;
                }
            }
        }
    }
}

void armor_detector::filter_lights(std::vector<cv::RotatedRect> &lights) {
    std::vector<cv::RotatedRect> light_rects;
    for (auto &light : lights) {
        double ratio = std::max(light.size.height, light.size.width) / std::min(light.size.height, light.size.width);
        double area = light.size.area();
        double angle = light.angle;
        if (ratio > 6 && area > 20 && std::abs(angle) < 10) {
            light_rects.push_back(light);
        } else if (ratio > 5 && area > 20 && std::abs(angle) < 10) {
            light_rects.push_back(light);
        } else if (ratio > 4 && area > 20 && std::abs(angle) < 15) {
            light_rects.push_back(light);
        } else if (ratio > 3.5 && area > 20 && std::abs(angle) < 15) {
            light_rects.push_back(light);
        } else if (ratio > 3.5 && area > 1000 && std::abs(angle) < 16) {
            light_rects.push_back(light);
        } else if (ratio > 3 && area > 100 && std::abs(angle) < 3) {
            light_rects.push_back(light);
        }
    }
    lights = light_rects;
}

void armor_detector::get_armors(std::vector<cv::RotatedRect> &lights, std::vector<cv::RotatedRect> &armors) {
    for (int i = 0; i < lights.size(); ++i) {
        for (int j = i + 1; j < lights.size(); ++j) {
            
        }
    }
}

void armor_detector::filter_armors(void) {

}

void armor_detector::init(xml_params_loader &_params) {
    enemy_color = (Enemy_Color)_params.enemy_color;

    color_thresh_val_blue = _params.color_thresh_val_blue;
    color_thresh_val_red = _params.color_thresh_val_red;
    brightness_thresh_val = _params.brightness_thresh_val;
}

}
}