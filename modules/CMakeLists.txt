set(CMAKE_CXX_STANDAND 17)

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

aux_source_directory(detect_armour detect_src)

add_library(modules SHARED ${detect_src})

target_link_libraries(modules
    ${OpenCV_LIBS}
    fmt::fmt
)
