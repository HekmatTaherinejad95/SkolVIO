include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

file(GLOB SOURCE_FILE
        "include/*.h"
        "src/*.cpp"
        )

set(TARGET "gtsam")

add_library(${TARGET} SHARED ${SOURCE_FILE})
target_link_libraries( ${TARGET}
        ${OpenCV_LIBS}
        camera_model
        feature_tracker
        mapdata )

target_include_directories(${TARGET} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        )
