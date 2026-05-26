#include <stdlib.h>
#include <glm/gtc/constants.hpp>
#include <geo/utils/logging/LogMacros.h>
#include <geo/math/ESA.h>
#include "EfficientICP.h"

namespace geo
{   
    ICPResult EfficientICP(const PointCloud3D& target, PointCloud3D& source, const PointCloud3D& subSource, 
        const INearestNeighbor& nn,
        const DistanceField& df, const EfficientICPParams& params)
    {
        TimePoint startTotal = Clock::now();

        // 1. Create the ESAParameters
        ESAParameters esa_parames = {};

        esa_parames.maxIterations = params.esaIterations;
        esa_parames.seed = params.seed;

        esa_parames.searchSpace = ESASearchSpace::FullRotation(target.ComputeBoundingBox());

        // 2. Run ESA of the subsample of source
        ESAResult esa_result = MultiStartESA(esa_parames, 
            [&](float* e) -> float {
            
            	const RigidTransform T = ConvertToRigidTransform({ e[0], e[1], e[2], e[3], e[4], e[5]});
            	const f32 maxDist = df.GetMaxDist();
            
            	f64 cost = 0.0;
            	u32 count = 0;
            
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
        
        // 3. Apply ESA found transform to source
        source.Transform(esa_result.transform);

        // 4. Run Sparse ICP starting from the ESA found point
        ICPResult result = geo::SparseICPPointToPlane(target, source, nn, params.icpParams);

        // 5. Combine results
        result.transform = RigidTransform::Compose(result.transform, esa_result.transform);

        // 4. Return result
        TimePoint endTotal = Clock::now();
        result.totalTime = TimeDifferenceMs(endTotal, startTotal);
        //result.totalESATime = esa_result.totalTime;

        return result;
    }

}