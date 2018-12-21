project(utility)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    /usr/local/include
    ${CURRENT_PROJECT_DIR}
)

add_library(utility
    xml_loader.cpp
)

target_link_libraries(utility
    ${OpenCV_LIBRARIES}
)