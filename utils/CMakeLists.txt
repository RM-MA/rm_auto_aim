set(CMAKE_CXX_STANDAND 17)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)

# include_directories("/home/ma/MA/2022-summer/utils/logger")
# link_directories("/home/ma/MA/2022-summer/utils/logger")

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

aux_source_directory(logger logger_src)
aux_source_directory(timer timer_src)

add_library(utils SHARED ${logger_src} ${timer_src})

target_link_libraries(utils
    ${OpenCV_LIBS}
    fmt::fmt
)
