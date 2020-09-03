cmake_minimum_required(VERSION 3.17)
project(SkolVIO)

set(CMAKE_CXX_STANDARD 14)


if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

add_subdirectory(${PROJECT_SOURCE_DIR}/src)


FIND_PACKAGE( OpenCV REQUIRED )

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(include)

enable_testing()	

add_subdirectory(tests)

