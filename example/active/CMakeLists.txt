cmake_minimum_required(VERSION 3.11)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(active)

# Build target module
add_executable(active main.cpp)
target_include_directories(active PRIVATE .)

target_include_directories(active PRIVATE ../../include)
target_include_directories(active PRIVATE ../../../s0s/include)
target_include_directories(active PRIVATE ../../../sl0/include)
target_include_directories(active PRIVATE ../../../../thirdparty/eigen)
