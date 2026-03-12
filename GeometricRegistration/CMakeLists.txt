cmake_minimum_required(VERSION 3.14)
project(GeometricRegistration)

# MultiThreaded compilation
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDebug$<$<CONFIG:Debug>:Debug>")
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Release>:Release>")
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE) #link time optimization 

# Define MY_SOURCES to be a list of all the source files for my Egnine 
file(GLOB_RECURSE MY_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/geo/*")
add_library("${PROJECT_NAME}" STATIC ${MY_SOURCES})
set_property(TARGET "${PROJECT_NAME}" PROPERTY CXX_STANDARD 17)

target_precompile_headers("${PROJECT_NAME}" PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/geo/core/pch.h")

if(NOT PRODUCTION_BUILD)
	# This is useful to get an ASSETS_PATH in your IDE during development
	target_compile_definitions("${PROJECT_NAME}" PUBLIC RESOURCES_PATH="${CMAKE_SOURCE_DIR}/resources/")
	target_compile_definitions("${PROJECT_NAME}" PUBLIC PRODUCTION_BUILD=0)
else()
	# setup the ASSETS_PATH macro to be in the root folder of your exe
	target_compile_definitions("${PROJECT_NAME}" PUBLIC RESOURCES_PATH="./resources/") 
	target_compile_definitions("${PROJECT_NAME}" PUBLIC PRODUCTION_BUILD=1) 
endif()

target_sources("${PROJECT_NAME}" PRIVATE ${MY_SOURCES} )

if(MSVC) # If using the VS compiler...
	target_compile_definitions("${PROJECT_NAME}" PUBLIC _CRT_SECURE_NO_WARNINGS)
	set_property(TARGET "${PROJECT_NAME}" PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreadedDebug<$<CONFIG:Debug>:Debug>")
	set_property(TARGET "${PROJECT_NAME}" PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Release>:Release>")

	if(PRODUCTION_BUILD)
		# no console
		set_target_properties("${PROJECT_NAME}" PROPERTIES WIN32_EXECUTABLE TRUE)
	endif()
endif()

target_include_directories("${PROJECT_NAME}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories("${PROJECT_NAME}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/geo")

############ link libraries ############

target_link_libraries("${PROJECT_NAME}" PRIVATE glm eigen nanoflann tiny_obj_loader)

########################################