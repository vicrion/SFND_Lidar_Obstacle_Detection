cmake_minimum_required(VERSION 2.8.12)

project(playback)

set(CXX_FLAGS "-Wall -std=c++17 -Wno-reorder")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

# Use dynamic linking for faster link times
set(BUILD_SHARED_LIBS ON)

# Optimization for debug builds
set(CMAKE_CXX_FLAGS_DEBUG "-Og -pipe -fdiagnostics-color=always -g1")

# Faster linking with gold linker or MSVC incremental linking
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(CMAKE_EXE_LINKER_FLAGS "-fuse-ld=gold")
elseif(MSVC)
    set(CMAKE_EXE_LINKER_FLAGS "/INCREMENTAL")
endif()

# Required dependencies
find_package(PCL REQUIRED COMPONENTS common io visualization segmentation)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} ${Boost_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp)
target_link_libraries (environment 
    ${PCL_LIBRARIES} 
    ${Boost_LIBRARIES} 
    spdlog::spdlog
    fmt::fmt
)

add_subdirectory(src/quiz/ransac)
