#include <opencv2/opencv.hpp>
#include "utility/xml_loader.hpp"

namespace Hero_Vision
{
namespace vision_mul
{

bool xml_params_loader::load(void) {
    cv::FileStorage fs(file_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cout << "Can't Open <" << file_path << ">" << std::endl;
        return false;
    }
    fs["video_path"] >> video_path;
    fs["enemy_color"] >> enemy_color;
    fs["color_thresh_val_blue"] >> color_thresh_val_blue;
    fs["color_thresh_val_red"] >> color_thresh_val_red;
    fs["brightness_thresh_val"] >> brightness_thresh_val;
}

}
}