#include "Model.h"

namespace tests
{
    Model::Model(const std::string& n, geo::Mesh m, geo::u32 dfResolution)
        :
        name(n),
        mesh(std::move(m)),
        cloud(mesh.ToPointCloud()),
        kdTree(cloud.GetPoints())
    {
        geo::DistanceFieldParameters dfParams;
        dfParams.bounding_box = mesh.BoundingBox();
        dfParams.resolution = dfResolution;

        // set the maximum radius to be 1/4 the diagonal of the mesh
        dfParams.max_distance = 0.25f * glm::length(dfParams.bounding_box.Max() - dfParams.bounding_box.Min());

        sdf = geo::DistanceField(dfParams);
        sdf.Build(mesh);
    }

    Model CreateModelFromOBJ(const std::filesystem::path& filePath, geo::u32 dfResolution)
    {
        std::string name = geo::io::GetFileName(filePath);
        geo::Mesh mesh = geo::Mesh::Load(filePath);

        return Model(name, std::move(mesh), dfResolution);
    }


}

