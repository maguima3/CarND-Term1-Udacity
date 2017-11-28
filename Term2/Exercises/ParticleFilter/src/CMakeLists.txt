project(PARTICLE_FILTER)

cmake_minimum_required(VERSION 2.8.11)

add_definitions(-std=c++0x)

set(SOURCES ./print_samples.cpp)

add_executable(particle_filter_exe ${SOURCES})
