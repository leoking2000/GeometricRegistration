cmake_minimum_required(VERSION 3.14)
project(stb)

add_library(stb)
target_sources(stb PRIVATE "src/stb.cpp")
target_include_directories(stb PUBLIC include)
