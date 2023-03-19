cmake_minimum_required(VERSION 3.13)    # CMake version check
project(cpp_example)                    # Create project "simple_example"
set(CMAKE_CXX_STANDARD 14)              # Enable c++14 standard

#Where to put generated file (executable, libraries...)!
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

add_executable( controller_example src/main.cpp src/control_loop.cpp )
target_link_libraries( controller_example boost_thread pthread )

