include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

file(GLOB SOURCE_FILE
    "include/*.hpp"
    "src/*.cpp"
)

set(TARGET "imu_integrator")

add_library(${TARGET} SHARED ${SOURCE_FILE})
target_link_libraries( ${TARGET}
                       ${EIGEN3_LIBS}
                       util )

target_include_directories(${TARGET} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(FILES ${headers} 
        DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/)
# Libraries
install(TARGETS ${TARGET}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
