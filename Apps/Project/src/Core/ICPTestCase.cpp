#include <glm/gtc/matrix_transform.hpp>
#include <geo/utils/GeoRand.h>
#include "ICPTestCase.h"

namespace test
{
    ICPTestCase KnowedTransform(const geo::PointCloud3D& target, const geo::RigidTransform& T, const std::string& name)
    {
        geo::PointCloud3D source = target;
        source.Transform(T);

        geo::DistanceField df;

        const geo::u32 resolution = 128;
        const geo::f32 dTrunc = 5.0f;
        const geo::f32 padding = 0.2f;

        df.Build(target, resolution, dTrunc, padding);

        return {
            name,
            target,
            source,
            df,
            T
        };
    }

    ICPTestCase PartialOverlap(const geo::PointCloud3D& target, const geo::RigidTransform& T,
        std::function<bool(const glm::vec3&)> keep, const std::string& name)
    {
        assert(keepRatio > 0.0f && keepRatio <= 1.0f);

        geo::PointCloud3D source = target;
        source.Transform(T);

        const auto& pts = source.GetPoints();
        const size_t N = pts.size();

        std::vector<glm::vec3> subset;
        subset.reserve(N);

        for (size_t i = 0; i < N; ++i)
        {
            if (keep(pts[i])) {
                subset.emplace_back(pts[i]);
            }
        }

        geo::PointCloud3D partialSource(subset);

        geo::DistanceField df;

        const geo::u32 resolution = 128;
        const geo::f32 dTrunc = 2.0f;
        const geo::f32 padding = 1.1f;

        df.Build(target, resolution, dTrunc, padding);

        return {
            name,
            target,
            partialSource,
            df,
            T
        };
    }


    ICPTestCase WithOutliers(const geo::PointCloud3D& target, const geo::RigidTransform& T,
        geo::u32 outlierCount, geo::f32 range, const std::string& name, geo::u32 seed)
    {
        assert(outlierCount >= 0);
        assert(range > 0.0f);

        geo::Random rng{ seed };

        geo::PointCloud3D source = target;
        source.Transform(T);

        std::vector<glm::vec3> pts = source.GetPoints();

        // Simple deterministic pseudo-random (no RNG dependency)
        for (int i = 0; i < outlierCount; ++i)
        {
            pts.emplace_back(rng.Float3(-range, range));
        }

        geo::PointCloud3D outlierSource(pts);

        geo::DistanceField df;

        const geo::u32 resolution = 128;
        const geo::f32 dTrunc = 2.0f;
        const geo::f32 padding = 1.1f;

        df.Build(target, resolution, dTrunc, padding);

        return {
            name,
            target,
            outlierSource,
            df,
            T
        };
    }

}