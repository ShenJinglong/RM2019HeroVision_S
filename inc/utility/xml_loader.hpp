#ifndef _XML_LOADER_H_
#define _XML_LOADER_H_

#include <opencv2/opencv.hpp>

namespace Hero_Vision
{
namespace vision_mul
{

class xml_params_loader {
public:
    xml_params_loader(std::string xml_file_path): file_path(xml_file_path) {}
    bool load(void);
public:
    std::string video_path;
    int enemy_color;
    double color_thresh_val_blue;
    double color_thresh_val_red;
    double brightness_thresh_val;
private:
    std::string file_path;
};

}
}

#endif