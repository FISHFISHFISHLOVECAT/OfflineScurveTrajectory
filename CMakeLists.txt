project(ScurvePlan)
cmake_minimum_required(VERSION 3.19.1)
find_package(Threads REQUIRED)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -pthread")
aux_source_directory(./src SRC)
include_directories(./inc)
add_executable(Demo ${SRC} )

