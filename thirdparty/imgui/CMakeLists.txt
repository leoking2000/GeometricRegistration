cmake_minimum_required(VERSION 3.14)
project(imgui)

# MultiThreaded compilation
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDebug$<$<CONFIG:Debug>:Debug>")
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Release>:Release>")
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE) #link time optimization

add_library(imgui)
target_sources(imgui PRIVATE 
"${CMAKE_CURRENT_SOURCE_DIR}/include/imgui/imgui.cpp"
"${CMAKE_CURRENT_SOURCE_DIR}/include/imgui/imgui_demo.cpp"
"${CMAKE_CURRENT_SOURCE_DIR}/include/imgui/imgui_draw.cpp"
"${CMAKE_CURRENT_SOURCE_DIR}/include/imgui/imgui_tables.cpp"
"${CMAKE_CURRENT_SOURCE_DIR}/include/imgui/imgui_widgets.cpp"
"${CMAKE_CURRENT_SOURCE_DIR}/include/imgui/rlImGui.cpp"
)

target_include_directories(imgui PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/")
target_include_directories(imgui PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../raylib/include/raylib")


target_link_libraries(imgui PRIVATE raylib)