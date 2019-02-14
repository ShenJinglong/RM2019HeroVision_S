project(common)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    /usr/local/include
    ${CURRENT_PROJECT_DIR}/inc
)

add_executable(hero_vision
    main.cpp
    vision.cpp
)

target_include_directories(hero_vision
    PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${CURRENT_PROJECT_DIR}/inc
)

link_directories(${CMAKE_LIBRARY_OUTPUT_DIR})

target_link_libraries(hero_vision
    ${OpenCV_LIBRARIES}
    detect
    tools
    utility
    driver
)