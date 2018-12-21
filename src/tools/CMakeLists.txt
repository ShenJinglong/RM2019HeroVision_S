project(tools)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    /usr/local/include
    ${CURRENT_PROJECT_DIR}
)

add_library(tools
    debug_tool.cpp
)

target_link_libraries(tools
    ${OpenCV_LIBRARIES}
)