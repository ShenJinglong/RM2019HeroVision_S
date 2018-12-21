#include <opencv2/opencv.hpp>
#include <thread>
#include "common/vision.hpp"

int main(int argc, char **argv) {
    std::thread vision_task(&Hero_Vision::vision_mul::vision_process);
    
    vision_task.join();
    return EXIT_SUCCESS;
}