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
        u32 esaIterations = 5000;
        u32 esaRestarts   = 5;
        u32 seed          = 2026;

        // --- ICP ---
        SparseICPParameters icpParams;
    };


    ICPResult EfficientICP(const PointCloud3D& target, PointCloud3D& source, const PointCloud3D& subSource, 
        const INearestNeighbor& nn,
        const DistanceField& df, const EfficientICPParams& params = {});
}