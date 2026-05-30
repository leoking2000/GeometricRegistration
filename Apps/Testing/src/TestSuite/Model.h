#pragma once
#include <string>
#include <functional>
#include <filesystem>
#include <geo/GeometricRegistration.h>

namespace tests
{
    // fully prebuilt model, shared across tests
    struct Model
    {
        Model(const std::string& n, geo::Mesh m, geo::u32 dfResolution);

        std::string        name;   // the name of the model
        geo::Mesh          mesh;   // full triangle mesh
        geo::PointCloud3D  cloud;  // all mesh vertices as a point cloud
        geo::DistanceField sdf;    // built from mesh  — for ESA cost
        geo::KDTree        kdTree; // built from cloud — for ICP closest-point

        // TODO: measure memory footprint
    };

    // Loads an OBJ and builds all spatial structures.
    // This is the only place Model is constructed from a file.
    Model CreateModelFromOBJ(const std::filesystem::path& filePath, geo::u32 dfResolution);
}
