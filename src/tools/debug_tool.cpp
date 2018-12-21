#include <opencv2/opencv.hpp>

#include "tools/debug_tool.hpp"

namespace Hero_Vision
{
namespace Tools
{

void draw_rotated_rect(cv::Mat &frame, const cv::RotatedRect &RRect) {
    cv::Point2f points[4];
    RRect.points(points);
    for (int i = 0; i < 4; ++i) {
        line(frame, points[i], points[(i + 1) % 4], cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }
}

void draw_rotated_rect_info(cv::Mat &frame, const cv::RotatedRect &RRect) {
    double ratio = std::max(RRect.size.height, RRect.size.width) / std::min(RRect.size.height, RRect.size.width);
    double area = RRect.size.area();
    double angle = RRect.angle;
    std::string text = cv::format("R: %f, A: %f, AN: %f", ratio, area, angle);
    cv::putText(frame, text, RRect.center, CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
}

void draw_rotated_rect_info(cv::Mat &frame_info_page, cv::Mat &frame_rect_page, const cv::RotatedRect &RRect, int index) {
    double ratio = std::max(RRect.size.height, RRect.size.width) / std::min(RRect.size.height, RRect.size.width);
    double area = RRect.size.area();
    double angle = RRect.angle;
    std::string text1 = cv::format("%d:: R: %f, A: %f, AN: %f", index, ratio, area, angle);
    std::string text2 = cv::format("%d", index);
    cv::putText(frame_info_page, text1, cv::Point(10, 20*(index+1)), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    cv::putText(frame_rect_page, text2, RRect.center, CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
}

void show_debug_info(const std::string &name, const cv::Size &size, std::vector<cv::RotatedRect> &RRects) {
    cv::Mat RRects_mat(size, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat RRects_info_mat(size, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int i = 0; i < RRects.size(); ++i) {
        draw_rotated_rect(RRects_mat, RRects[i]);
        draw_rotated_rect_info(RRects_info_mat, RRects_mat, RRects[i], i);
    }
    imshow(name + "_RRects", RRects_mat);
    imshow(name + "_infos", RRects_info_mat);
}

}
}