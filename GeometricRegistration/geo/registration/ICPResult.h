#pragma once
#include "utils/GeoTypes.h"
#include "utils/GeoTime.h"
#include "math/RigidTransform.h"

namespace geo
{
    struct ICPResult
    {
        RigidTransform transform = {};
        u32 iterations = 0;
        bool converged = false;
        f64 rmse = 0.0;
        TimingStat totalIterationTime;
        TimingStat correspondenceSearchTime;
        TimingStat alignmentSolveTime;
    };
}