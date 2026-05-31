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

        // leaves mesh and sdf empty!
        Model(const std::string& n, geo::PointCloud3D cloud);

        Model(const Model&) = delete;
        Model& operator=(const Model&) = delete;
        Model(Model&&) = default;
        Model& operator=(Model&&) = default;

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
