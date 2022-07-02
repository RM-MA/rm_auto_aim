cmake_minimum_required(VERSION 3.15)

project(ma)
set(CMAKE_CXX_STANDAND 17)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)


message("init----------------------------")
message("Eigen: Version=${Eigen3_VERSION}")
message("OpenCV: Version=${OpenCV_VERSION}")
message("end-----------------------------")


add_subdirectory(utils)
link_directories(utils)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
add_executable(fmt_test fmt_test.cpp)




target_link_libraries(fmt_test 
${OpenCV_LIBS}
fmt::fmt)
