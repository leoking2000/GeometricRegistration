#pragma once
#include "INearestNeighbor.h"
#include "PointCloud3D.h"
#include "RigidTransform.h"

namespace geo
{
    struct ICPResult
    {
        RigidTransform transform = {};
        float rms = 0.0f;
        int iterations = 0;
        bool converged = false;
        double totalElapsed_ms = 0.0;
        double avgCorrespondenceTime_ms = 0.0f;
        double avgSolverTime_ms = 0.0f;
    };

    /**
     * @brief Performs point-to-point ICP (Iterative Closest Point) alignment between two 3D point clouds.
     *
     * This is the "naive" implementation using brute-force closest point search.
     * It iteratively:
     *   1. Finds the closest point in the target cloud for each source point.
     *   2. Computes the optimal rigid transform (rotation + translation) using SVD.
     *   3. Applies the transform to the source cloud.
     *   4. Computes RMS error and checks for convergence.
     *
     * Timing measurements are collected for:
     *   - Correspondence search (correspondencesTime_ms)
     *   - Solver computation (solverTime_ms)
     *   - Total elapsed time (totalElapsed_ms)
     *
     * @param targetSurface The fixed target surface.
     * @param source The source point cloud that will be transformed in-place.
     * @param maxIterations Maximum number of ICP iterations.
     * @param tolerance Convergence tolerance on RMS change between iterations.
     * @return ICPResult Contains the final transform, RMS error, iteration count, convergence flag, and timing.
     *
     * @note This implementation is O(n^2) in number of points due to brute-force nearest neighbor search.
     */
    ICPResult NaiveICP(const INearestNeighbor& targetSurface, PointCloud3D& source, int maxIterations, float tolerance = 1e-5f);
}
