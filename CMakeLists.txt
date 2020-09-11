cmake_minimum_required(VERSION 3.17)
set(PROJECT_NAME SkolVIO)
set(CMAKE_CXX_STANDARD 14)
project( ${PROJECT_NAME})


find_package( OpenCV REQUIRED )


if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( Ceres REQUIRED)

include_directories(
		${OpenCV_INCLUDE_DIRS}
		${EIGEN3_INCLUDE_DIR}
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(util)
add_subdirectory(dataset_loader)
#add_subdirectory(mapdata)
add_subdirectory(camera_model)
#add_subdirectory(simulator)
add_subdirectory(feature_tracker)
add_subdirectory(imu_integrator)
#add_subdirectory(orientation_tracking)
#add_subdirectory(multiview_helper)
#add_subdirectory(map_initializer)
#add_subdirectory(graph_optimizer)
#add_subdirectory(pnp_estimator)
#add_subdirectory(visual_inertial_odometry)
#add_subdirectory(back-end)


include_directories(include)

enable_testing()	



