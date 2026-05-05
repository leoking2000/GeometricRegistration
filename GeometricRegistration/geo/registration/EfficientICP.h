#include <geo/geometry/PointCloud3D.h>
#include <geo/spatial/INearestNeighbor.h>
#include <geo/spatial/DistanceField.h>
#include "SparseICP.h"
#include "ICPResult.h"

namespace geo
{
    struct EfficientICPParams
    {
        // --- ESA ---
        u32 esaIterations = 2000;
        u32 esaRestarts   = 5;

        // --- ICP ---
        SparseICPParameters icpParams;
    };


    ICPResult EfficientICP(const PointCloud3D& target, PointCloud3D& source, const INearestNeighbor& nn,
        const DistanceField& df, const EfficientICPParams& params = {});
}