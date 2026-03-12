cmake_minimum_required(VERSION 3.14)
project(tiny_obj_loader)

# MultiThreaded compilation
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDebug$<$<CONFIG:Debug>:Debug>")
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Release>:Release>")
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE) #link time optimization

add_library(tiny_obj_loader INTERFACE)


target_include_directories(tiny_obj_loader INTERFACE include "${CMAKE_CURRENT_SOURCE_DIR}/")
