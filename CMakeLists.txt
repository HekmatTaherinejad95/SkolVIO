cmake_minimum_required(VERSION 3.17)
project(SkolVIO)

set(CMAKE_CXX_STANDARD 14)
FIND_PACKAGE( OpenCV REQUIRED )

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(include)


add_executable(SkolVIO src/main.cpp)
