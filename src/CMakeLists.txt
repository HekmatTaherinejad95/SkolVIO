cmake_minimum_required(VERSION 3.17)
project(SkolVIO)
add_executable(SkolVIO main.cpp)
add_executable(VO VO.cpp)
find_package( OpenCV REQUIRED )
find_package(GTSAM REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

target_link_libraries( SkolVIO ${OpenCV_LIBS}
        gtsam )

target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${GTSAM_INCLUDE_DIR}
        )


