cmake_minimum_required(VERSION 3.20)
project(cows)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_OSX_ARCHITECTURES arm64)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(cows main.cpp CloudHandler.cpp CloudHandler.hpp)
target_link_libraries(cows ${PCL_LIBRARIES} ${OpenCV_LIBS})