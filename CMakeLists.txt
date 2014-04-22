cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project( opencv_tests )

find_package( PCL 1.2 REQUIRED)
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.

add_library(stereoVision src/stereovision.cpp)
add_library(parametersForSGBM src/parametersforsgbm.cpp)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


find_package( OpenCV REQUIRED )
add_executable(
opencv_tests src/main.cpp



)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.45.0 COMPONENTS thread date_time program_options filesystem system)

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})

endif()

target_link_libraries( opencv_tests ${OpenCV_LIBS} ${PCL_LIBRARIES} stereoVision parametersForSGBM ${Boost_LIBRARIES})

