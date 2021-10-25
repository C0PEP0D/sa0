cmake_minimum_required(VERSION 3.11)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(agent)

# Build target module
add_executable(agent main.cpp)
target_include_directories(agent PRIVATE .)

target_include_directories(agent PRIVATE ../../include)
target_include_directories(agent PRIVATE ../../../s0s/include)
target_include_directories(agent PRIVATE ../../../sl0/include)
target_include_directories(agent PRIVATE ../../../../thirdparty/eigen)
