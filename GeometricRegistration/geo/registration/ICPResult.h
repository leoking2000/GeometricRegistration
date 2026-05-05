#pragma once
#include "utils/GeoTypes.h"
#include "utils/GeoTime.h"
#include "math/RigidTransform.h"

namespace geo
{
    struct ICPResult
    {
        RigidTransform transform = {};
        f64 totalTime = 0.0f;

        // ICP Results
        u32  iterations = 0;
        bool converged = false;
        f32  rmse = 0.0;

        // ICP time stats
        TimingStat totalIterationTime;
        TimingStat correspondenceSearchTime;
        TimingStat alignmentSolveTime;

        //ESA
        TimingStat totalESATime;
    };
}
