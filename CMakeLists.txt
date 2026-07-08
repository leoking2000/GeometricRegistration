cmake_minimum_required(VERSION 3.14)
project(ALL_BUILD)
set(CMAKE_CXX_STANDARD 17)

#DELETE THE OUT FOLDER AFTER CHANGING THIS BECAUSE VISUAL STUDIO DOESN'T SEEM TO RECOGNIZE THIS CHANGE AND REBUILD!
option(PRODUCTION_BUILD "Make this a production build" OFF)
#DELETE THE OUT FOLDER AFTER CHANGING THIS BECAUSE VISUAL STUDIO DOESN'T SEEM TO RECOGNIZE THIS CHANGE AND REBUILD!

find_package(OpenMP REQUIRED)

find_package(OpenMP)
if(NOT OpenMP_CXX_FOUND)
    message(WARNING "OpenMP not found")
endif()

###### Add Libraries ######

add_subdirectory(thirdparty/glm)             # math library
add_subdirectory(thirdparty/Eigen)           # Eigen linear algebra
add_subdirectory(thirdparty/nanoflann)       # Nearest Neighbor (NN) search
add_subdirectory(thirdparty/tiny_obj_loader) # obj loader

add_subdirectory(thirdparty/googletest)      # unit tests
                                             
add_subdirectory(thirdparty/glad)            # OpenGL loader
add_subdirectory(thirdparty/glfw-3.3.2)      # window management
add_subdirectory(thirdparty/stb)             # texture read-write

##################################

add_subdirectory(Libraries/Geo)              # static library
add_subdirectory(Apps/Testing)               # runs tests
add_subdirectory(Apps/ViewerApp)             # visualization tool
add_subdirectory(Apps/GeoAlign)              # CLI tool
