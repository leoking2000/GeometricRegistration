#pragma once
#include <geo/geometry/PointCloud3D.h>
#include <geo/spatial/INearestNeighbor.h>
#include "ICPResult.h"

namespace geo
{
    // Parameters controlling Sparse ICP optimization.
    struct SparseICPParameters
    {
        // Maximum number of ICP iterations before forced termination.
        u32 maxIterations = 500;

        // RMSE convergence threshold.
        // Optimization stops when the RMSE improvement falls below this value.
        f32 tolerance = 1e-5f;

        // Translation convergence threshold.
        // Measured in world-space units.
        f32 transTolerance = 1e-5f;

        // Rotation convergence threshold in radians.
        // Used to detect very small rotational updates.
        f32 rotTolerance = 1e-4f;

        f32 p = 0.4f;                  // Sparsity exponent.
        f32 mu = 10;                   // ADMM penalty parameter.
        u32 admmIterations = 10;       // Number of inner ADMM iterations per ICP iteration.
    };

    // Sparse point-to-point ICP.
    //
    // target:
    //   Fixed/reference point cloud.
    //
    // source:
    //   Moving point cloud to align onto target.
    //
    // nn:
    //   Nearest-neighbor structure built over target points.
    //
    // params:
    //   Sparse ICP optimization parameters.
    //
    // Returns:
    //   Final transform, RMSE, convergence state, and timing statistics.
    ICPResult SparseICPPointToPoint(
        const PointCloud3D& target, const PointCloud3D& source, 
        const INearestNeighbor& nn, SparseICPParameters params = {});

    // Sparse point-to-plane ICP.
    //
    // Uses target normals to minimize point-to-plane residuals.
    // Typically converges faster and more accurately on smooth surfaces.
    //
    // target:
    //   Fixed/reference point cloud with normals.
    //
    // source:
    //   Moving point cloud to align onto target.
    //
    // nn:
    //   Nearest-neighbor structure built over target points.
    //
    // params:
    //   Sparse ICP optimization parameters.
    //
    // Returns:
    //   Final transform, RMSE, convergence state, and timing statistics.
    ICPResult SparseICPPointToPlane(
        const PointCloud3D& target, const PointCloud3D& source, 
        const INearestNeighbor& nn, SparseICPParameters params = {});
}
