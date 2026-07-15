cmake_minimum_required(VERSION 3.14)
project(Core)

# MultiThreaded compilation
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDebug$<$<CONFIG:Debug>:Debug>")
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Release>:Release>")
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE) #link time optimization

# Define MY_SOURCES to be a list of all the source files for my Library 
file(GLOB_RECURSE MY_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/core/*")
add_library("${PROJECT_NAME}" STATIC ${MY_SOURCES})
set_property(TARGET "${PROJECT_NAME}" PROPERTY CXX_STANDARD 17)

target_sources("${PROJECT_NAME}" PRIVATE ${MY_SOURCES} )

if(MSVC) # If using the VS compiler...
	target_compile_definitions("${PROJECT_NAME}" PUBLIC _CRT_SECURE_NO_WARNINGS)
	set_property(TARGET "${PROJECT_NAME}" PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreadedDebug<$<CONFIG:Debug>:Debug>")
	set_property(TARGET "${PROJECT_NAME}" PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Release>:Release>")
endif()

target_include_directories("${PROJECT_NAME}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories("${PROJECT_NAME}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/core")
target_include_directories("${PROJECT_NAME}" PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/core")

if(OpenMP_CXX_FOUND)
    target_link_libraries("${PROJECT_NAME}" PUBLIC  OpenMP::OpenMP_CXX)
else()
    message(WARNING "OpenMP not found, building without it")
endif()

if(MSVC)
    target_compile_options("${PROJECT_NAME}" PUBLIC /openmp)
endif()

############ link libraries ############

target_link_libraries("${PROJECT_NAME}" PRIVATE glm tiny_obj_loader stb)

########################################
