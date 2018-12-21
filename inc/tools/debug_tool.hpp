#ifndef _DEBUG_TOOL_
#define _DEBUG_TOOL_

#include <opencv2/opencv.hpp>

namespace Hero_Vision
{
namespace Tools
{

void draw_rotated_rect(cv::Mat &frame, const cv::RotatedRect &RRect);
void draw_rotated_rect_info(cv::Mat &frame, const cv::RotatedRect &RRect);
void draw_rotated_rect_info(cv::Mat &frame_info_page, cv::Mat &frame_rect_page, const cv::RotatedRect &RRect, int index);
void show_debug_info(const std::string &name, const cv::Size &size, std::vector<cv::RotatedRect> &RRects);

}
}

#endif