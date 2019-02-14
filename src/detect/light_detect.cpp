#include <opencv2/opencv.hpp>
#include "detect/light_detect.hpp"

namespace Hero_Vision
{
namespace vision_mul
{

void Light_Detector::detect_lights(cv::Mat &frame, std::vector<cv::RotatedRect> &lights) const {
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

    for (auto &light : lights) {
        if (light.size.width < light.size.height) {
            continue;
        } else {
            double temp;
            temp = light.size.height;
            light.size.height = light.size.width;
            light.size.width = temp;
            light.angle += 90;
        }
    }
}

void Light_Detector::filter_lights(std::vector<cv::RotatedRect> &lights) {
    std::vector<cv::RotatedRect> light_rects;
    for (auto &light : lights) {
        double ratio = std::max(light.size.height, light.size.width) / std::min(light.size.height, light.size.width);
        double area = light.size.area();
        double angle = light.angle;
        if (3 < ratio && ratio < 13 &&
            100 < area && area < 1700 &&
            std::abs(angle) < 15) {
                light_rects.push_back(light);
        } else if (0) {
                light_rects.push_back(light);
        }
    }
    lights = light_rects;
}

}
}