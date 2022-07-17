cmake_minimum_required(VERSION 3.15)

set(CMAKE_CXX_STANDAND 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)

aux_source_directory(serial serial_src)

aux_source_directory(camera mv_camera_src)

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(devices SHARED ${mv_camera_src} ${serial_src})


target_link_libraries(devices
    ${OpenCV_LIBS}
    fmt::fmt
    MVSDK
)