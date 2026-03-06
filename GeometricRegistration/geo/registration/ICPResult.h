#pragma once
#include "geo/math/RigidTransform.h"

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
}