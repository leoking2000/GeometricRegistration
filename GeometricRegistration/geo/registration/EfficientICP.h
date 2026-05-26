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
        u32 esaRestarts   = 5;     // Number of independent ESA restarts.
        u32 seed          = 2026;  // Random seed controlling ESA stochastic behavior. 

        // --- ICP ---
        SparseICPParameters icpParams; // Parameters forwarded directly to Sparse ICP solver.
    };

    // Runs a hybrid global-to-local registration pipeline:
    //
    // 1. Uses ESA (Enhanced Simulated Annealing) to explore the SE(3) space
    //    and find a good initial rigid transformation.
    //
    // 2. Refines the best candidate using Sparse ICP, which performs
    //    robust correspondence-based optimization.
    //
    // 3. Uses a sub-sampled source ("subSource") for global search efficiency,
    //    while full source is used for final ICP refinement.
    //
    // Inputs:
    //  - target: full target point cloud (fixed)
    //  - source: full source point cloud (to be aligned)
    //  - subSource: reduced/filtered version of source for ESA speedup
    //  - nn: nearest neighbor structure for correspondence search
    //  - df: distance field for fast spatial queries / cost evaluation
    //  - params: configuration for ESA + ICP stages
    //
    // Output:
    //  - ICPResult containing final transform, RMSE, convergence info,
    //    and timing statistics for each stage.
    ICPResult EfficientICP(const PointCloud3D& target, const PointCloud3D& source, const PointCloud3D& subSource, 
        const INearestNeighbor& nn, const DistanceField& df, const EfficientICPParams& params = {});
}