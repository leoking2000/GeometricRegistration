cmake_minimum_required(VERSION 3.14)
project(glm)

# MultiThreaded compilation
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDebug$<$<CONFIG:Debug>:Debug>")
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Release>:Release>")
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE) #link time optimization

add_library(glm INTERFACE)


target_include_directories(glm INTERFACE include "${CMAKE_CURRENT_SOURCE_DIR}/")
