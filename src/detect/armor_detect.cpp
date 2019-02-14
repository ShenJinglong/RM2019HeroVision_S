#include <opencv2/opencv.hpp>

#include "detect/armor_detect.hpp"
#include "tools/debug_tool.hpp"
#include "utility/xml_loader.hpp"

#define POINT_DIST(p1,p2) std::sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y))

namespace Hero_Vision
{
namespace vision_mul
{

Armor_info boundingRRect(const cv::RotatedRect & left, const cv::RotatedRect & right){
    Armor_info single_armor_info;
	const cv::Point & pl = left.center, & pr = right.center;
	cv::Point2f center; 
	center.x = (pl.x + pr.x) / 2.0;
	center.y = (pl.y + pr.y) / 2.0;
	cv::Size2f wh_l = left.size;
	cv::Size2f wh_r = right.size;
	float width = POINT_DIST(pl, pr) - (wh_l.width + wh_r.width) / 2.0;
	float height = std::max(wh_l.height, wh_r.height);
	float angle = std::atan2(right.center.y - left.center.y, right.center.x - left.center.x);
	single_armor_info.armor_RRect = cv::RotatedRect(center, cv::Size2f(width, height), angle * 180 / CV_PI);
    single_armor_info.armor_points.left_bottom = cv::Point2f(left.center.x - left.size.height / 2 * std::sin(left.angle * CV_PI / 180), left.center.y + left.size.height / 2 * std::cos(left.angle * CV_PI / 180));
    single_armor_info.armor_points.left_top = cv::Point2f(left.center.x + left.size.height / 2 * std::sin(left.angle * CV_PI / 180), left.center.y - left.size.height / 2 * std::cos(left.angle * CV_PI / 180));
    single_armor_info.armor_points.right_bottom = cv::Point2f(right.center.x - right.size.height / 2 * std::sin(right.angle * CV_PI / 180), right.center.y + right.size.height / 2 * std::cos(right.angle * CV_PI / 180));
    single_armor_info.armor_points.right_top = cv::Point2f(right.center.x + right.size.height / 2 * std::sin(right.angle * CV_PI / 180), right.center.y - right.size.height / 2 * std::cos(right.angle * CV_PI / 180));
    return single_armor_info;
}

void Armor_Detector::get_armors(std::vector<cv::RotatedRect> &lights, std::vector<Armor_info> &armors) {
    if (lights.size() >= 2) {
        for (int i = 0; i < lights.size() - 1; ++i) {
            for (int j = i + 1; j < lights.size(); ++j) {
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
                    /*
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
                    */                
                }
            }
        }
    }
}

void Armor_Detector::filter_armors(void) {

}

}
}