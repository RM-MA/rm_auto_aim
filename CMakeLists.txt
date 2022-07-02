cmake_minimum_required(VERSION 3.15)

project(ma)
set(CMAKE_CXX_STANDAND 17)

find_package(OpenCV REQUIRED)
link_directories(${OpenCV_DIRS})
add_executable(main main.cpp)

target_link_libraries(main ${OpenCV_LIBS})
