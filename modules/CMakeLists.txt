set(CMAKE_CXX_STANDAND 17)

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

aux_source_directory(kalman_filter kalman_src)
aux_source_directory(detect_armour detect_src)
aux_source_directory(number_classifier number_classifier_src)
# aux_source_directory(posture_calculating posture_calculating_src)

message("${kalman_src}")
add_library(modules SHARED ${kalman_src} ${detect_src} ${number_classifier_src})

target_link_libraries(modules
    ${OpenCV_LIBS}
    fmt::fmt
)
