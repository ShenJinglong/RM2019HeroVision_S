project(detect)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    /usr/local/include
    ${CURRENT_PROJECT_DIR}/inc
)

add_library(detect
    armor_detect.cpp    
)

target_link_libraries(detect
    ${OpenCV_LIBRARIES}
)