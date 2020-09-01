include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

file(GLOB SOURCE_FILE
    "include/*.hpp"
    "src/*.cpp"
)

set(TARGET "feature_tracker")

add_library(${TARGET} SHARED ${SOURCE_FILE})
target_link_libraries( ${TARGET} 
                       ${OpenCV_LIBS} 
                       camera_model
                       mapdata )

target_include_directories(${TARGET} PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include 
)

# Add application
add_executable(${TARGET}_app ${TARGET}_app.cpp)
target_link_libraries(${TARGET}_app ${TARGET} mapdata util )

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp")
# Headers
# TODO: Change to use {PROJECT_NAME}/{TARGET} as include directory.
install(FILES ${headers} 
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME})
# Libraries
install(TARGETS ${TARGET}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
# Executables
install(TARGETS ${TARGET}_app
  EXPORT ${PROJECT_NAME}Targets
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin )



if (USE_TEST)
  file(GLOB TEST_SOURCE_FILE
      "test/*.cpp" )
       
  add_executable(${TARGET}_test ${TEST_SOURCE_FILE})
  target_link_libraries(${TARGET}_test ${TARGET}
    gtest gtest_main
    util mapdata
    ${OpenCV_LIBS} )
  add_test( FeatureTrackerTest ${TARGET}_test )
endif()
