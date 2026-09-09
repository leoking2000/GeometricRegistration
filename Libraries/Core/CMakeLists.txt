cmake_minimum_required(VERSION 3.14)
project(Core VERSION 1.0.0 LANGUAGES CXX)

############ project files ############

# Define CORE_HEADERS and CORE_SOURCES to be a list of all the source files for my Library
file(GLOB_RECURSE CORE_HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/core/*.h")
file(GLOB_RECURSE CORE_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/core/*.cpp")
add_library("${PROJECT_NAME}" STATIC ${CORE_HEADERS} ${CORE_SOURCES})

# we are using c++17
target_compile_features("${PROJECT_NAME}" PUBLIC cxx_std_17)

############ include directories ############

target_include_directories("${PROJECT_NAME}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

############ PRODUCTION_BUILD Defines ############

if(PRODUCTION_BUILD)
	target_compile_definitions("${PROJECT_NAME}" PUBLIC PRODUCTION_BUILD=1)
else()
	target_compile_definitions("${PROJECT_NAME}" PUBLIC PRODUCTION_BUILD=0) 
endif()

############ MSVC optimazations ############

if(MSVC) # If using the VS compiler...
	target_compile_definitions("${PROJECT_NAME}" PUBLIC _CRT_SECURE_NO_WARNINGS)
	set_property(TARGET "${PROJECT_NAME}" PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
	set_property(TARGET "${PROJECT_NAME}" PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Release>:Release>")
endif()

############ OpenMP ############

find_package(OpenMP)

if(OpenMP_FOUND)
    target_link_libraries(Core PUBLIC OpenMP::OpenMP_CXX)
else()
    message(WARNING "OpenMP not found, building without it")
endif()

############ link libraries ############

target_link_libraries("${PROJECT_NAME}" PUBLIC glm PRIVATE tiny_obj_loader tiny_gltf_loader stb)

########################################
