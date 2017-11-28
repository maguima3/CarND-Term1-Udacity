cmake_minimum_required(VERSION 2.8.11)
project(1D_FILTER)

# Use C++
set(SOURCES main.cpp
	    bayesianFilter.cpp)

set_source_files_properties(${SOURCES} PROPERTIES COMPILE_FLAGS -std=c++0x)

# Create the executable
add_executable(1d_markov_loc ${SOURCES})

