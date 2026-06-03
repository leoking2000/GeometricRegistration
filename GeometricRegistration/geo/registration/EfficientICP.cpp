#define GLM_ENABLE_EXPERIMENTAL
#include <stdlib.h>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/quaternion.hpp>
#include <geo/utils/logging/LogMacros.h>
#include <geo/math/ESA.h>
#include "EfficientICP.h"

namespace geo
{   
    static inline std::vector<glm::vec3> GenerateIcosahedronFaceDirections()
    {
        constexpr float phi = 1.6180339887498948482f;

        std::vector<glm::vec3> V =
        {
            {-1,  phi,  0},
            { 1,  phi,  0},
            {-1, -phi,  0},
            { 1, -phi,  0},

            { 0, -1,  phi},
            { 0,  1,  phi},
            { 0, -1, -phi},
            { 0,  1, -phi},

            { phi,  0, -1},
            { phi,  0,  1},
            {-phi,  0, -1},
            {-phi,  0,  1}
        };

        for (auto& v : V)
            v = glm::normalize(v);

        static const int F[20][3] =
        {
            {0,11,5}, {0,5,1}, {0,1,7}, {0,7,10}, {0,10,11},
            {1,5,9}, {5,11,4}, {11,10,2}, {10,7,6}, {7,1,8},
            {3,9,4}, {3,4,2}, {3,2,6}, {3,6,8}, {3,8,9},
            {4,9,5}, {2,4,11}, {6,2,10}, {8,6,7}, {9,8,1}
        };

        std::vector<glm::vec3> dirs;
        dirs.reserve(20);

        for (int i = 0; i < 20; i++)
        {
            glm::vec3 c =
                (V[F[i][0]] +
                    V[F[i][1]] +
                    V[F[i][2]]) / 3.0f;

            dirs.push_back(glm::normalize(c));
        }

        return dirs;
    }

    static inline std::vector<ESAParameters> GenerateESAConfigurations(const ESAParameters& baseParams)
    {
        std::vector<ESAParameters> configs;

        const auto directions = GenerateIcosahedronFaceDirections();

        configs.reserve(directions.size());

        const glm::vec3 referenceDir(0.0f, 0.0f, 1.0f);

        for (geo::u32 i = 0; i < (geo::u32)directions.size(); i++)
        {
            ESAParameters p = baseParams;

            const glm::vec3& dir = directions[i];

            glm::quat q = glm::rotation(referenceDir, dir);

            glm::vec3 euler = glm::eulerAngles(q);

            p.initialTransform =
            {
                0.0f,           // tx
                0.0f,           // ty
                0.0f,           // tz

                euler.x,        // rx
                euler.y,        // ry
                euler.z         // rz
            };

            p.seed = baseParams.seed ^ (i * 2654435761u);

            configs.push_back(p);
        }

        return configs;
    }

    EfficientICPResult EfficientICP(const PointCloud3D& target, const PointCloud3D& source, const PointCloud3D& subSource,
        const INearestNeighbor& nn,
        const DistanceField& df, const EfficientICPParams& params)
    {
        // Create mutable working copy of source cloud.
        // ICP progressively transforms this cloud toward the target.
        PointCloud3D src = source;

        TimePoint startTotal = Clock::now();

        // ------------------------------------------------------------
        // 1. Configure ESA global optimization parameters
        // ------------------------------------------------------------

        ESAParameters esa_parames = {};

        esa_parames.maxIterations = params.esaIterations;
        esa_parames.seed = params.seed;
        esa_parames.searchSpace = ESASearchSpace::FullRotation(target.ComputeBoundingBox());

        std::vector<ESAParameters> configs = GenerateESAConfigurations(esa_parames);

        // ------------------------------------------------------------
        // 2. Run ESA on a subsampled source cloud
        // ------------------------------------------------------------
        ESAResult esa_result = MultiConfigESA(configs,
            [&](float* e) -> float {
            
            	const RigidTransform T = ConvertToRigidTransform({ e[0], e[1], e[2], e[3], e[4], e[5]});
            	const f32 maxDist = df.GetMaxDist();
            
            	f64 cost = 0.0;
            	u32 count = 0;
            
                // We use subSource instead of full source to reduce cost of
                // expensive distance-field evaluations during global search.
            	for (index_t i = 0; i < subSource.Size(); i++)
            	{
            		glm::vec3 tp = T.TransformPoint(subSource.Point(i));
            		f32 d = glm::abs(df(tp));
            
            		if (d < maxDist)
            		{
            			cost += glm::pow(d, params.icpParams.p);
            			count++;
            		}
            	}
            
            	if (count == 0) {
            		return maxDist * maxDist;
            	}
            
            	return float(cost / (f64)count);
            });
        
        // ------------------------------------------------------------
        // 3. Apply ESA result as initial alignment
        // ------------------------------------------------------------

        src.Transform(esa_result.transform);

        // ------------------------------------------------------------
        // 4. Local refinement using Sparse ICP
        // ------------------------------------------------------------

        ICPResult icp_result = geo::SparseICPPointToPlane(target, src, nn, params.icpParams);

        // ------------------------------------------------------------
        // 5. Combine global + local transforms
        // ------------------------------------------------------------

        geo::RigidTransform transform = RigidTransform::Compose(icp_result.transform, esa_result.transform);

        // ------------------------------------------------------------
        // 6. Total timing
        // ------------------------------------------------------------

        TimePoint endTotal = Clock::now();

        return { transform, icp_result, esa_result, TimeDifferenceMs(endTotal, startTotal)};
    }

}