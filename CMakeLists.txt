cmake_minimum_required(VERSION 3.17)
project(SkolVIO)

set(CMAKE_CXX_STANDARD 14)
FIND_PACKAGE( OpenCV REQUIRED )


add_executable(SkolVIO main.cpp)
