#include <stdlib.h>
#include <glm/gtc/constants.hpp>
#include <geo/utils/logging/LogMacros.h>
#include <geo/math/ESA.h>
#include "EfficientICP.h"

namespace geo
{   
    ICPResult EfficientICP(const PointCloud3D& target, const PointCloud3D& source, const PointCloud3D& subSource, 
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

        // ------------------------------------------------------------
        // 2. Run ESA on a subsampled source cloud
        // ------------------------------------------------------------
        ESAResult esa_result = MultiStartESA(esa_parames, 
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
            		f32 d = df(tp);
            
            		if (glm::abs(d) < maxDist)
            		{
            			cost += glm::pow(glm::abs(d), params.icpParams.p);
            			count++;
            		}
            	}
            
            	if (count == 0) {
            		return maxDist * maxDist;
            	}
            
            	return float(cost / (f64)count);
            }
            , params.esaRestarts);
        
        // ------------------------------------------------------------
        // 3. Apply ESA result as initial alignment
        // ------------------------------------------------------------

        src.Transform(esa_result.transform);

        // ------------------------------------------------------------
        // 4. Local refinement using Sparse ICP
        // ------------------------------------------------------------

        ICPResult result = geo::SparseICPPointToPlane(target, src, nn, params.icpParams);

        // ------------------------------------------------------------
        // 5. Combine global + local transforms
        // ------------------------------------------------------------

        result.transform = RigidTransform::Compose(result.transform, esa_result.transform);

        // ------------------------------------------------------------
        // 6. Total timing
        // ------------------------------------------------------------

        TimePoint endTotal = Clock::now();
        result.totalTimeMs = TimeDifferenceMs(endTotal, startTotal);

        return result;
    }

}