#include <opencv2/opencv.hpp>

#include "detect/armor_detect.hpp"
#include "tools/debug_tool.hpp"
#include "utility/xml_loader.hpp"

#define __DEBUG_LIGHT__
#define __DEBUG_ARMOR__
#define POINT_DIST(p1,p2) std::sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y))

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

#ifdef __DEBUG_LIGHT__
    Tools::show_debug_info("show_lights_after_detect", frame.size(), lights);
#endif

    filter_lights(lights);

#ifdef __DEBUG_LIGHT__
    cv::Mat show_lights_after_filter(frame.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    for (auto &light : lights) {
        Tools::draw_rotated_rect(show_lights_after_filter, light);
    }
    for (int i = 0; i < lights.size(); ++i) {
        cv::putText(show_lights_after_filter, cv::format("%d", i), lights[i].center, CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    }
    imshow("show_lights_after_filter", show_lights_after_filter);
#endif

    get_armors(lights, armors);

#ifdef __DEBUG_ARMOR__
    cv::Mat show_armors_after_get(frame.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    for (auto &armor : armors) {
        Tools::draw_rotated_rect(show_armors_after_get, armor);
    }
    imshow("show_armors_after_get", show_armors_after_get);
#endif

    filter_armors();
}

void armor_detector::detect_lights(cv::Mat &frame, std::vector<cv::RotatedRect> &lights) const {
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
        /*
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
        }*/
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

cv::RotatedRect boundingRRect(const cv::RotatedRect & left, const cv::RotatedRect & right){
	const cv::Point & pl = left.center, & pr = right.center;
	cv::Point2f center; 
	center.x = (pl.x + pr.x) / 2.0;
	center.y = (pl.y + pr.y) / 2.0;
	cv::Size2f wh_l = left.size;
	cv::Size2f wh_r = right.size;
	float width = POINT_DIST(pl, pr) - (wh_l.width + wh_r.width) / 2.0;
	float height = std::max(wh_l.height, wh_r.height);
	float angle = std::atan2(right.center.y - left.center.y, right.center.x - left.center.x);
	return cv::RotatedRect(center, cv::Size2f(width, height), angle * 180 / CV_PI);
}

void armor_detector::get_armors(std::vector<cv::RotatedRect> &lights, std::vector<cv::RotatedRect> &armors) {
    if (lights.size() >= 2) {
        for (int i = 0; i < lights.size() - 1; ++i) {
            for (int j = i + 1; j < lights.size(); ++j) {
                /*
                if (std::abs(lights[i].center.x - lights[j].center.x) < 4 * std::max(lights[i].size.height, lights[j].size.height) &&
                    std::abs(lights[i].center.y - lights[j].center.y) < 1.5 * std::max(lights[i].size.height, lights[j].size.height)) {
                    double light_ratio_i = std::max(lights[i].size.width, lights[i].size.height) /
                                            std::min(lights[i].size.width, lights[i].size.height);
                    double light_ratio_j = std::max(lights[j].size.width, lights[j].size.height) /
                                            std::min(lights[j].size.width, lights[j].size.height);
                    double light_ratio_r = std::max(light_ratio_i, light_ratio_j) /
                                            std::min(light_ratio_i, light_ratio_j) - 1;
                    double light_area_r = std::max(lights[i].size.area(), lights[j].size.area()) /
                                            std::min(lights[i].size.area(), lights[j].size.area()) - 1;
                    double light_angle_r = std::max(std::abs(lights[i].angle), std::abs(lights[j].angle)) /
                                            std::min(std::abs(lights[i].angle), std::abs(lights[j].angle)) - 1;
                    double judge_val = light_ratio_r * light_weight_ratio + light_angle_r * light_weight_angle
                                        + light_area_r * light_weight_area;
                    if (judge_val < 6) {
                        armors.push_back(lights[i].center.x < lights[j].center.x ? boundingRRect(lights[i], lights[j]) : 
                                                                        boundingRRect(lights[j], lights[i]));
                    }
                }
                */
                double height_diff = std::abs(lights[i].size.height - lights[j].size.height);
                double height_sum = lights[i].size.height + lights[j].size.height;
                double width_diff = std::abs(lights[i].size.width - lights[j].size.width);
                double width_sum = lights[i].size.width + lights[j].size.width;
                double angle_diff = std::abs(lights[i].angle - lights[j].angle);
                double Y_diff = std::abs(lights[i].center.y - lights[j].center.y);
                double MH_diff = std::min(lights[i].size.height, lights[j].size.height) * 2 / 3;
                double height_max = std::max(lights[i].size.height, lights[j].size.height);
                double X_diff = std::abs(lights[i].center.x - lights[j].center.x);
                
                double T_ANGLE_THRE = 5, T_ANGLE_THRE180 = 3, T_ANGLE_THREMIN = 3, T_ANGLE_THRE180MIN = 2;
                double T_HIGH_RAT = 0.2, T_WIDTH_RAT = 0.4;
                double T_HIGH_RAT_ANGLE = 0.34, T_WIDTH_RAT_ANGLE = 0.55;

                if (
                    Y_diff < MH_diff && X_diff < height_max * 4 &&
                    (angle_diff < T_ANGLE_THRE || 180 - angle_diff < T_ANGLE_THRE180) &&
                    height_diff / height_sum < T_HIGH_RAT &&
                    width_diff / width_sum < T_WIDTH_RAT
                ) {
                    /*
                    std::cout << cv::format("11111111111111111111111~~%d--%d~~11111111111111111111111111", i, j) << std::endl;
                    std::cout << "height_diff: " << height_diff << std::endl;
                    std::cout << "height_sum: " << height_sum << std::endl;
                    std::cout << "width_diff: " << width_diff << std::endl;
                    std::cout << "width_sum: " << width_sum << std::endl;
                    std::cout << "angle_diff: " << angle_diff << std::endl;
                    std::cout << "Y_diff: " << Y_diff << std::endl;
                    std::cout << "MH_diff: " << MH_diff << std::endl;
                    std::cout << "height_max: " << height_max << std::endl;
                    std::cout << "X_diff: " << X_diff << std::endl;
                    std::cout << "111111111111111111111111111111111111111111111111111111111111" << std::endl;
                    */
                    armors.push_back(lights[i].center.x < lights[j].center.x ? boundingRRect(lights[i], lights[j]) : boundingRRect(lights[j], lights[i]));
                } else if (
                    (angle_diff < T_ANGLE_THREMIN || 180 - angle_diff < T_ANGLE_THRE180MIN) &&
                    Y_diff < MH_diff * 3 / 2 && X_diff < height_max * 4 &&
                    height_diff / height_sum < T_HIGH_RAT_ANGLE &&
                    width_diff / width_sum < T_WIDTH_RAT_ANGLE
                ) {
                    /*
                    std::cout << cv::format("2222222222222222222222222222~~%d--%d~~2222222222222222222222222", i, j) << std::endl;
                    std::cout << "height_diff: " << height_diff << std::endl;
                    std::cout << "height_sum: " << height_sum << std::endl;
                    std::cout << "width_diff: " << width_diff << std::endl;
                    std::cout << "width_sum: " << width_sum << std::endl;
                    std::cout << "angle_diff: " << angle_diff << std::endl;
                    std::cout << "Y_diff: " << Y_diff << std::endl;
                    std::cout << "MH_diff: " << MH_diff << std::endl;
                    std::cout << "height_max: " << height_max << std::endl;
                    std::cout << "X_diff: " << X_diff << std::endl;
                    std::cout << "2222222222222222222222222222222222222222222222222" << std::endl;
                    */
                    armors.push_back(lights[i].center.x < lights[j].center.x ? boundingRRect(lights[i], lights[j]) : boundingRRect(lights[j], lights[i]));
                } else if (
                    (angle_diff < 3 || 180 - angle_diff < 2) &&
                    Y_diff < MH_diff * 2 && X_diff < height_max * 4
                ) {
                    /*
                    std::cout << cv::format("33333333333333333333333~~%d--%d~~33333333333333333333333333", i, j) << std::endl;
                    std::cout << "height_diff: " << height_diff << std::endl;
                    std::cout << "height_sum: " << height_sum << std::endl;
                    std::cout << "width_diff: " << width_diff << std::endl;
                    std::cout << "width_sum: " << width_sum << std::endl;
                    std::cout << "angle_diff: " << angle_diff << std::endl;
                    std::cout << "Y_diff: " << Y_diff << std::endl;
                    std::cout << "MH_diff: " << MH_diff << std::endl;
                    std::cout << "height_max: " << height_max << std::endl;
                    std::cout << "X_diff: " << X_diff << std::endl;
                    std::cout << "3333333333333333333333333333333333333333333333333" << std::endl;
                    */
                    armors.push_back(lights[i].center.x < lights[j].center.x ? boundingRRect(lights[i], lights[j]) : boundingRRect(lights[j], lights[i]));
                } else if (
                    (angle_diff < 3 || 180 - angle_diff < 2) &&
                    Y_diff < MH_diff * 3 && X_diff < height_max * 5
                ) {
                    /*
                    std::cout << cv::format("4444444444444444444444444444444~~%d--%d~~4444444444444444444444444444", i, j) << std::endl;
                    std::cout << "height_diff: " << height_diff << std::endl;
                    std::cout << "height_sum: " << height_sum << std::endl;
                    std::cout << "width_diff: " << width_diff << std::endl;
                    std::cout << "width_sum: " << width_sum << std::endl;
                    std::cout << "angle_diff: " << angle_diff << std::endl;
                    std::cout << "Y_diff: " << Y_diff << std::endl;
                    std::cout << "MH_diff: " << MH_diff << std::endl;
                    std::cout << "height_max: " << height_max << std::endl;
                    std::cout << "X_diff: " << X_diff << std::endl;
                    std::cout << "444444444444444444444444444444444444444444444444444444444" << std::endl;
                    */
                    armors.push_back(lights[i].center.x < lights[j].center.x ? boundingRRect(lights[i], lights[j]) : boundingRRect(lights[j], lights[i]));
                } else if (
                    (angle_diff < 6.5 || 180 - angle_diff < 2) &&
                    Y_diff < MH_diff && X_diff < height_max * 5
                ) {
                    /*
                    std::cout << cv::format("5555555555555555555555555555~~%d--%d~~5555555555555555555555555", i, j) << std::endl;
                    std::cout << "height_diff: " << height_diff << std::endl;
                    std::cout << "height_sum: " << height_sum << std::endl;
                    std::cout << "width_diff: " << width_diff << std::endl;
                    std::cout << "width_sum: " << width_sum << std::endl;
                    std::cout << "angle_diff: " << angle_diff << std::endl;
                    std::cout << "Y_diff: " << Y_diff << std::endl;
                    std::cout << "MH_diff: " << MH_diff << std::endl;
                    std::cout << "height_max: " << height_max << std::endl;
                    std::cout << "X_diff: " << X_diff << std::endl;
                    std::cout << "55555555555555555555555555555555555555555555555555555555555555" << std::endl;
                    */
                    armors.push_back(lights[i].center.x < lights[j].center.x ? boundingRRect(lights[i], lights[j]) : boundingRRect(lights[j], lights[i]));
                } else {
                    std::cout << cv::format("********************%d--%d*********************", i, j) << std::endl;
                    std::cout << "height_diff: " << height_diff << std::endl;
                    std::cout << "height_sum: " << height_sum << std::endl;
                    std::cout << "width_diff: " << width_diff << std::endl;
                    std::cout << "width_sum: " << width_sum << std::endl;
                    std::cout << "angle_diff: " << angle_diff << std::endl;
                    std::cout << "Y_diff: " << Y_diff << std::endl;
                    std::cout << "MH_diff: " << MH_diff << std::endl;
                    std::cout << "height_max: " << height_max << std::endl;
                    std::cout << "X_diff: " << X_diff << std::endl;
                    std::cout << "*********************************************************" << std::endl;
                }
            }
        }
    }
}

void armor_detector::filter_armors(void) {

}

}
}