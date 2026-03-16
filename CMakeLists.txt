cmake_minimum_required(VERSION 3.14)
project(ALL_BUILD)
set(CMAKE_CXX_STANDARD 17)

#DELETE THE OUT FOLDER AFTER CHANGING THIS BECAUSE VISUAL STUDIO DOESN'T SEEM TO RECOGNIZE THIS CHANGE AND REBUILD!
option(PRODUCTION_BUILD "Make this a production build" OFF)
#DELETE THE OUT FOLDER AFTER CHANGING THIS BECAUSE VISUAL STUDIO DOESN'T SEEM TO RECOGNIZE THIS CHANGE AND REBUILD!

###### Add Libraries ######

add_subdirectory(thirdparty/Eigen)           # Eigen linear algebra
add_subdirectory(thirdparty/nanoflann)       # Nearest Neighbor (NN) search
add_subdirectory(thirdparty/glm)             # math library
add_subdirectory(thirdparty/tiny_obj_loader)

add_subdirectory(thirdparty/googletest)     # unit tests

add_subdirectory(thirdparty/raylib)         # rendering/ui library for Visualization


##################################

add_subdirectory(GeometricRegistration)
add_subdirectory(Apps/Project)
add_subdirectory(Apps/Testing)