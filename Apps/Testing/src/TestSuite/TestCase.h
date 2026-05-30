#pragma once
#include "Model.h"

namespace tests
{
    struct TestCase
    {
        std::string name = "unnamed";

        Model* target;
        Model* source;

        geo::RigidTransform groundTruth; // moves source to target

        // metadata about this test case.
        geo::f32 overlapRatio = 1.0f;
        geo::f32 outlierRatio = 0.0f;
        geo::f32 noiseStdDev  = 0.0f;
    };
}
