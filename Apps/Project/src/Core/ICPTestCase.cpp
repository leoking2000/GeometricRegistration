#include <glm/gtc/matrix_transform.hpp>
#include <geo/utils/GeoRand.h>
#include "ICPTestCase.h"

namespace test
{
    Model CreateModelFromOBJ(const std::filesystem::path& filePath, const geo::DistanceFieldParameters& df_params)
    {
        Model m = {};
        m.name = geo::io::GetFileName(filePath);
        m.mesh = geo::Mesh::Load(filePath);
        m.cloud = m.mesh.ToPointCloud();
        m.kdTree = geo::KDTree(m.cloud.GetPoints());
        m.sdf = geo::DistanceField(df_params);
        m.sdf.Build(m.mesh);
        return m;
    }

    // Internal helpers //
    
    // Gaussian noise via Box-Muller
    static inline glm::vec3 GaussianNoise(const glm::vec3& p, geo::f32 stdDev, geo::Random& rng)
    {
        auto sample = [&]() -> geo::f32
            {
                geo::f32 u1 = rng.Float(1e-6f, 1.0f);
                geo::f32 u2 = rng.Float(0.0f, 1.0f);
                return std::sqrt(-2.0f * std::log(u1)) * std::cos(glm::two_pi<geo::f32>() * u2);
            };
        return p + glm::vec3(sample(), sample(), sample()) * stdDev;
    }

    // Partial overlap — keep overlapRatio fraction of points from
    // the "front" of the object (+X half-space in local frame).
    // The rest are back-face points with no target correspondence.
    static inline void ApplyPartialOverlap(
        std::vector<glm::vec3>& pts,
        std::vector<glm::vec3>& nrm,
        geo::f32 overlapRatio)
    {
        if (overlapRatio >= 1.0f || pts.empty()) return;

        const bool hasNormals = (nrm.size() == pts.size());

        std::vector<geo::f32> proj(pts.size());
        for (size_t i = 0; i < pts.size(); i++)
            proj[i] = pts[i].x;

        std::vector<geo::f32> sorted = proj;
        std::sort(sorted.begin(), sorted.end());

        const size_t keepCount =
            std::max(size_t(1), size_t(std::round(pts.size() * overlapRatio)));
        const geo::f32 threshold = sorted[keepCount - 1];

        std::vector<glm::vec3> front, back, frontN, backN;
        for (size_t i = 0; i < pts.size(); i++)
        {
            if (proj[i] <= threshold) {
                front.push_back(pts[i]);
                if (hasNormals) frontN.push_back(nrm[i]);
            }
            else {
                back.push_back(pts[i]);
                if (hasNormals) backN.push_back(nrm[i]);
            }
        }

        pts.clear(); nrm.clear();
        pts.insert(pts.end(), front.begin(), front.end());
        pts.insert(pts.end(), back.begin(), back.end());
        if (hasNormals) {
            nrm.insert(nrm.end(), frontN.begin(), frontN.end());
            nrm.insert(nrm.end(), backN.begin(), backN.end());
        }
    }

    // Outliers — replace tail points with random noise in bbox
    static inline void ApplyOutliers(
        std::vector<glm::vec3>& pts,
        std::vector<glm::vec3>& nrm,
        geo::f32 outlierRatio,
        const geo::BBox& bbox,
        geo::Random& rng)
    {
        if (outlierRatio <= 0.0f || pts.empty()) return;

        const bool hasNormals = (nrm.size() == pts.size());
        const glm::vec3 expand = bbox.Size() * 0.2f;
        const glm::vec3 bmin = bbox.Min() - expand;
        const glm::vec3 bmax = bbox.Max() + expand;

        const size_t count = size_t(std::round(pts.size() * outlierRatio));
        const size_t startIdx = pts.size() - count;

        for (size_t i = startIdx; i < pts.size(); i++) {
            pts[i] = rng.Float3(0.0f, 1.0f) * (bmax - bmin) + bmin;
            if (hasNormals) nrm[i] = rng.Dir3D(1.0f);
        }
    }

    static inline std::unique_ptr<Model> BuildSourceModel(
        const geo::Mesh& targetMesh,
        const TestCaseParams& params)
    {
        geo::Random rng(params.seed);

        // Sample from target mesh surface
        geo::PointCloud3D raw = targetMesh.SamplePointsUniform(params.sampleRatio * targetMesh.VertexCount(), rng, true);


    }



}