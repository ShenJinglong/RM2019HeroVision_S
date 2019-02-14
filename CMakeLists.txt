cmake_minimum_required(VERSION 3.1)

project(hero_vision)

set(CMAKE_BUILD_TYPE RELEASE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

find_package(OpenCV REQUIRED)

set(CURRENT_PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CURRENT_PROJECT_DIR}/build)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CURRENT_PROJECT_DIR}/build/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CURRENT_PROJECT_DIR}/build/lib)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/common)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/detect)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/tools)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/utility)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/driver)