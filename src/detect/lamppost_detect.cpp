#include <opencv2/opencv.hpp>
#include "tools/debug_tool.hpp"
#include "detect/lamppost_detect.hpp"
#include "detect/armor_detect.hpp"

//#define __DEBUG_LAMPPOST_DETECT_RESULT__
//#define __DEBUG_SHOW_HP__

namespace Hero_Vision 
{
namespace vision_mul
{

void Lamppost_Detector::detect_lights(cv::Mat &frame, std::vector<cv::RotatedRect> &maybe_lampposts_rect) const {
    cv::Mat frame_binary;
    cv::inRange(frame, cv::Scalar(245, 245, 245), cv::Scalar(255, 255, 255), frame_binary);

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(frame_binary, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    for (auto &contour : contours) {
        cv::RotatedRect RRect = cv::minAreaRect(contour);
        maybe_lampposts_rect.push_back(RRect);
    }
}

void Lamppost_Detector::get_lamppost(cv::Mat &frame, std::vector<Armor_info> armors, std::vector<lamppost_info> lampposts) {
    for (const auto &armor : armors) {
        double armor_ratio = std::max(armor.armor_RRect.size.width, armor.armor_RRect.size.height) / std::min(armor.armor_RRect.size.width, armor.armor_RRect.size.height);
        if (armor_ratio < 3) {
            continue;
        }

        cv::RotatedRect maybe_lamppost;
        maybe_lamppost.center.x = armor.armor_RRect.center.x + (armor.armor_RRect.size.height * 2.2) * std::sin(armor.armor_RRect.angle / 180 * CV_PI);
        maybe_lamppost.center.y = armor.armor_RRect.center.y - (armor.armor_RRect.size.height * 2.2) * std::cos(armor.armor_RRect.angle / 180 * CV_PI);
        maybe_lamppost.size.height = armor.armor_RRect.size.height;
        maybe_lamppost.size.width = armor.armor_RRect.size.width;
        maybe_lamppost.angle = armor.armor_RRect.angle;

        // 旋转
        cv::Mat rot_dstImage;
        cv::Mat rot_mat = cv::getRotationMatrix2D(maybe_lamppost.center, maybe_lamppost.angle, 1);
        cv::warpAffine(frame, rot_dstImage, rot_mat, frame.size());
        
        // 仿射变换
        cv::Point2f srcTri[3];
        cv::Point2f dstTri[3];
        srcTri[0] = cv::Point2f(maybe_lamppost.center.x - maybe_lamppost.size.width / 2, maybe_lamppost.center.y - maybe_lamppost.size.height / 2);
        srcTri[1] = cv::Point2f(maybe_lamppost.center.x - maybe_lamppost.size.width / 2, maybe_lamppost.center.y + maybe_lamppost.size.height / 2);
        srcTri[2] = cv::Point2f(maybe_lamppost.center.x + maybe_lamppost.size.width / 2, maybe_lamppost.center.y + maybe_lamppost.size.height / 2);
        dstTri[0] = cv::Point2f(0, 0);
        dstTri[1] = cv::Point2f(0, 20);
        dstTri[2] = cv::Point2f(100, 20);                
        cv::Mat warp_dstImage;
        cv::Mat warp_mat = cv::getAffineTransform(srcTri, dstTri);
        cv::warpAffine(rot_dstImage, warp_dstImage, warp_mat, warp_dstImage.size());
        
        cv::Mat lamppost_roi = warp_dstImage(cv::Rect(0, 0, 100, 20));

        std::vector<cv::RotatedRect> maybe_lampposts_rect;

        detect_lights(lamppost_roi, maybe_lampposts_rect);

        if (maybe_lampposts_rect.empty()) {
            continue;
        }

        std::sort(maybe_lampposts_rect.begin(), maybe_lampposts_rect.end(), [](const cv::RotatedRect &RRect1, const cv::RotatedRect &RRect2) { return RRect1.size.area() < RRect2.size.area(); });

        cv::RotatedRect lamppost = maybe_lampposts_rect[maybe_lampposts_rect.size()-1];

        if (lamppost.size.width < lamppost.size.height) {
            double temp = lamppost.size.width;
            lamppost.size.width = lamppost.size.height;
            lamppost.size.height = temp;
            lamppost.angle += 90;
        }

#ifdef __DEBUG_LAMPPOST_DETECT_RESULT__
        Tools::draw_rotated_rect(lamppost_roi, lamppost);
        cv::imshow("lamppost_roi", lamppost_roi);
#endif

        double HP = lamppost.size.width / (lamppost_roi.size().width * 0.84);

#ifdef __DEBUG_SHOW_HP__       
        std::cout << HP << std::endl;
        cv::putText(frame, cv::format("%f%%", HP * 100), maybe_lamppost.center, CV_FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
#endif

        struct lamppost_info single_lamppost;
        single_lamppost.HP = HP;
        single_lamppost.lamppost_rect = maybe_lamppost;

        lampposts.push_back(single_lamppost);
        
        /*
        cv::Point2f armor2lamppost_vector;
        armor2lamppost_vector.x = armor.armor_RRect.size.height * 2.2 * std::sin(armor.armor_RRect.angle / 180 * CV_PI);
        armor2lamppost_vector.y = -1 * armor.armor_RRect.size.height * 2.2 * std::cos(armor.armor_RRect.angle / 180 * CV_PI);
        
        cv::Point2f srcTri[3];
        cv::Point2f dstTri[3];
        
        srcTri[0] = armor.armor_points.left_top + armor2lamppost_vector;
        srcTri[1] = armor.armor_points.left_bottom + armor2lamppost_vector;
        srcTri[2] = armor.armor_points.right_bottom + armor2lamppost_vector;

        
        cv::circle(frame, srcTri[0], 2, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        cv::circle(frame, srcTri[1], 2, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        cv::circle(frame, srcTri[2], 2, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        cv::circle(frame, armor.armor_points.right_top + armor2lamppost_vector, 2, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        

        dstTri[0] = cv::Point2f(0, 0);
        dstTri[1] = cv::Point2f(0, 20);
        dstTri[2] = cv::Point2f(100, 20);                
        cv::Mat warp_dstImage;
        cv::Mat warp_mat = cv::getAffineTransform(srcTri, dstTri);
        cv::warpAffine(frame, warp_dstImage, warp_mat, warp_dstImage.size());

        cv::Mat lamppost_roi = warp_dstImage(cv::Rect(0, 0, 100, 20));

        std::vector<cv::RotatedRect> lights;

        cv::inRange(lamppost_roi, cv::Scalar(245, 245, 245), cv::Scalar(255, 255, 255), lamppost_roi);

        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(lamppost_roi, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

        std::vector<cv::RotatedRect> maybe_lampposts_rect;

        for (auto &contour : contours) {
            cv::RotatedRect RRect = cv::minAreaRect(contour);
            maybe_lampposts_rect.push_back(RRect);
        }

        if (maybe_lampposts_rect.empty()) {
            continue;
        }

        std::sort(maybe_lampposts_rect.begin(), maybe_lampposts_rect.end(), [](const cv::RotatedRect &RRect1, const cv::RotatedRect &RRect2) { return RRect1.size.area() < RRect2.size.area(); });

        cv::RotatedRect lamppost = maybe_lampposts_rect[maybe_lampposts_rect.size()-1];
        cv::cvtColor(lamppost_roi, lamppost_roi, cv::COLOR_GRAY2BGR);

        Tools::draw_rotated_rect(lamppost_roi, lamppost);

        if (lamppost.size.width < lamppost.size.height) {
            double temp = lamppost.size.width;
            lamppost.size.width = lamppost.size.height;
            lamppost.size.height = temp;
            lamppost.angle += 90;
        }

        double HP = lamppost.size.width / (lamppost_roi.size().width * 0.84);
        std::cout << HP << std::endl;

        struct lamppost_info single_lamppost;
        single_lamppost.HP = HP;

        cv::RotatedRect lamppost_RRect;
        lamppost_RRect.center.x = armor.armor_RRect.center.x + (armor.armor_RRect.size.height * 2.2) * std::sin(armor.armor_RRect.angle / 180 * CV_PI);
        lamppost_RRect.center.y = armor.armor_RRect.center.y - (armor.armor_RRect.size.height * 2.2) * std::cos(armor.armor_RRect.angle / 180 * CV_PI);
        lamppost_RRect.size.height = armor.armor_RRect.size.height;
        lamppost_RRect.size.width = armor.armor_RRect.size.width;
        lamppost_RRect.angle = armor.armor_RRect.angle;

        single_lamppost.lamppost_rect = lamppost_RRect;

        lampposts.push_back(single_lamppost);

        cv::putText(frame, cv::format("%f%%", HP*100), lamppost_RRect.center, CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        cv::imshow("lamppost_roi", lamppost_roi);
        */
    }
}

}
}