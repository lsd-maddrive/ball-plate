cmake_minimum_required(VERSION 3.0)
project(plane_control)

set (CMAKE_CXX_STANDARD 11)
set(Boost_USE_MULTITHREADED      ON)

find_package(Boost REQUIRED COMPONENTS
                program_options
                )

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} pthread)
