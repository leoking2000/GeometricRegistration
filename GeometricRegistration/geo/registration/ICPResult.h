#pragma once
#include <geo/utils/GeoTime.h>
#include <geo/math/RigidTransform.h>

namespace geo
{
    struct ICPResult
    {
        // Final rigid transform estimated by ICP.
        // Maps source geometry into target space.
        RigidTransform transform = {};

        f64 totalTimeMs = 0.0;  // Total wall-clock execution time in milliseconds.

        // --- ICP convergence results ---

        u32  iterations = 0;       // Number of ICP iterations executed.
        bool converged  = false;   // True if convergence criterion was satisfied.
        f32  rmse       = 0.0f;    // Final registration RMSE after optimization.

        // --- Timing breakdown statistics ---

        // Full iteration timing statistics.
        // Includes correspondence search + solve + transform update.
        TimingStat totalIterationTime;

        // Time spent finding nearest-neighbor correspondences.
        TimingStat correspondenceSearchTime;

        // Time spent solving alignment system
        TimingStat alignmentSolveTime;
    };
}
