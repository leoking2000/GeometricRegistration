#pragma once
#include "core/GeoTypes.h"
#include "math/RigidTransform.h"

namespace geo
{
    struct ICPResult
    {
        RigidTransform transform = {};
        f32 rms = 0.0f;
        u32 iterations = 0;
        bool converged = false;
        f64 totalElapsed_ms = 0.0;
        f64 avgCorrespondenceTime_ms = 0.0;
        f64 avgSolverTime_ms = 0.0;
    };
}