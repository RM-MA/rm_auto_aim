cmake_minimum_required(VERSION 3.15)

project(ma)
set(CMAKE_CXX_STANDAND 17)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # clangd
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Ofast -pthread" )

# 添加cmake模块以使用ceres库
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

message("init----------------------------")
message("Eigen: Version=${Eigen3_VERSION}")
message("OpenCV: Version=${OpenCV_VERSION}")
message("Ceres: Version=${CERES_VERSION}")
message("end-----------------------------")


include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})

# add_executable(fmt_test fmt_test.cpp)

# add_executable(configs_test configs_test.cpp)

add_executable(serial_test serial_test.cpp)

# add_executable(create_mat create_mat.cpp)

# add_executable(shoot_test shoot_test.cpp)

# add_executable(kf_test kf_test.cpp)

# add_executable(ekf_test ekf_test.cpp)

# target_link_libraries(fmt_test 
# ${OpenCV_LIBS}
# fmt::fmt)

# target_link_libraries(configs_test
# ${OpenCV_LIBS}
# fmt::fmt)

# target_link_libraries(create_mat 
# ${OpenCV_LIBS}
# fmt::fmt)

# target_link_libraries(shoot_test
# ${OpenCV_LIBS}
# fmt::fmt)

# target_link_libraries(kf_test
# ${OpenCV_LIBS}
# fmt::fmt)


# target_link_libraries(ekf_test
# fmt::fmt
# ${OpenCV_LIBS}
# ${CERES_LIBRARIES}
# )

add_executable(eigen_test eigen_test.cpp)