cmake_minimum_required (VERSION 3.13.0)
project (uavEE)

set (CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -static-libstdc++ -std=c++1z")

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system)


if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif (NOT CMAKE_BUILD_TYPE)


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/extern)
include_directories(${UAVGS_INCLUDE_DIR})


add_subdirectory(src)


install(DIRECTORY include/uavEE DESTINATION include)
install(DIRECTORY config DESTINATION config/uavEE)
