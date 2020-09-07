# For building shared lib
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

file(GLOB SOURCE_FILE
    "include/*.hpp"
    "src/*.cpp"
)

# TODO: Find other way than use as library
set(TARGET "util")

add_library(${TARGET} SHARED ${SOURCE_FILE})

target_include_directories(${TARGET} PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include 
)


file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp")

install(FILES ${headers} 
        DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/)
# Libraries
install(TARGETS ${TARGET}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )

if (USE_TEST)
  file(GLOB TEST_SOURCE_FILES
    "test/*.cpp" )
  foreach (TEST_SOURCE_FILE ${TEST_SOURCE_FILES})
    get_filename_component(TEST_NAME ${TEST_SOURCE_FILE} NAME_WE)

    add_executable(${TEST_NAME} ${TEST_SOURCE_FILE})
    # TODO: Maybe better to define libs for each test
    target_link_libraries(${TEST_NAME} ${TARGET}
      gtest gtest_main
      ${OpenCV_LIBS} )
    add_test( ${TARGET}Test ${TEST_NAME} )
  endforeach()
endif()
