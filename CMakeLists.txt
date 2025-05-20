cmake_minimum_required(VERSION 3.10)
project(VersionProject VERSION 1.0)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED) # Add pkgconfig

pkg_check_modules(LIBCAMERA REQUIRED libcamera) # find libcamera using pkg-config

include_directories("/usr/include/opencv4")
if(LIBCAMERA_FOUND) # only include if found.
    include_directories(${LIBCAMERA_INCLUDE_DIRS})
endif()
include_directories("/usr/lib/aarch64-linux-gnu")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(Tracking main_tracking.cpp tracking.cpp)
add_executable(Calibration main_calibration.cpp calibration.cpp)

target_link_libraries(Calibration
    ${OpenCV_LIBS}
    cam2opencv
    ${LIBCAMERA_LIBRARIES} # link against libcamera libraries
)

target_link_libraries(Tracking
    ${OpenCV_LIBS}
    cam2opencv
    ${LIBCAMERA_LIBRARIES} # link against libcamera libraries
)
