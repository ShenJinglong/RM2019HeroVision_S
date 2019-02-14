project(driver)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    /usr/local/include
    ${CURRENT_PROJECT_DIR}/inc
)

add_library(driver
    camera_driver.cpp    
)

target_link_libraries(driver
    ${OpenCV_LIBRARIES}
)