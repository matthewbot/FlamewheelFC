cmake_minimum_required(VERSION 2.8)

include_directories(include src)
add_definitions(-std=gnu++11 -g3 -ffast-math)

file(GLOB_RECURSE sources src/*.cpp)
add_executable(fc.elf ${sources})
