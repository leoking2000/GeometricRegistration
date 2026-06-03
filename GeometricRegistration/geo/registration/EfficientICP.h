#include <geo/geometry/PointCloud3D.h>
#include <geo/spatial/INearestNeighbor.h>
#include <geo/spatial/DistanceField.h>
#include "SparseICP.h"
#include "ICPResult.h"

namespace geo
{
    // Parameters controlling the Efficient ICP pipeline.
    // This method typically combines:
    //  - Global search (ESA) to find a good initial alignment
    //  - Local refinement (Sparse ICP) to converge to a precise solution
    struct EfficientICPParams
    {
        // --- ESA ---
        u32 esaIterations = 5000;  // Maximum number of ESA iterations per run.
        u32 seed          = 2026;  // Random seed controlling ESA stochastic behavior. 

        // --- ICP ---
        SparseICPParameters icpParams; // Parameters forwarded directly to Sparse ICP solver.
    };

    struct EfficientICPResult
    {
        geo::RigidTransform transform;
        ICPResult icp_result;
        ESAResult esa_result;

        f64 totalTime = 0.0;
    };

    // Runs a hybrid global-to-local registration pipeline, Runs ESA, then does Sparse ICP
    // Inputs:
    //  - target: full target point cloud (fixed)
    //  - source: full source point cloud (to be aligned)
    //  - subSource: reduced/filtered version of source for ESA speedup
    //  - nn: nearest neighbor structure for correspondence search in ICP
    //  - df: distance field for fast spatial queries / cost evaluation for ESA
    //  - params: configuration for ESA + ICP stages
    EfficientICPResult EfficientICP(
        const PointCloud3D& target, const PointCloud3D& source, const PointCloud3D& subSource,
        const INearestNeighbor& nn, const DistanceField& df, const EfficientICPParams& params = {});
}