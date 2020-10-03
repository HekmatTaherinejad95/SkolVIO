cmake_minimum_required(VERSION 2.8)

project( examples )

find_package( OpenCV REQUIRED )
find_package( OpenGL )

if(OPENGL_FOUND)
    include_directories( ${OpenCV_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIRS} )
else()
    include_directories( ${OpenCV_INCLUDE_DIRS} )
endif()

add_executable(Featuretrackingexample Featuretrackingexample.cpp)

target_link_libraries(Featuretrackingexample ${OpenCV_LIBRARIES} ${OPENGL_LIBRARIES})



