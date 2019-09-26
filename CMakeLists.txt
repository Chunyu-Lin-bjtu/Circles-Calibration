cmake_minimum_required(VERSION 2.8.11)

PROJECT(CIRCLECALIBRATION)


set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(Opencv_FOUND 1)
set(Opencv_DIR /home/yaoyishen/opencv-2.4.13.6/build)
find_package( Opencv REQUIRED )
include_directories(${Opencv_INCLUDE_DIRS})


add_executable(circlecalib circlecalib.cpp)
target_link_libraries(circlecalib ${OpenCV_LIBS} "-lpopt")


