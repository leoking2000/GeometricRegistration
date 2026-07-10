#pragma once
#include "Model.h"

namespace tests
{
    struct TestCase
    {
        std::string name = "unnamed";

        Model* target = nullptr;
        Model* source = nullptr;

        core::RigidTransform groundTruth; // moves source to target

        // metadata about this test case.
        f32 sampleRatio  = -1.0f;
        f32 overlapRatio = 1.0f;
        f32 outlierRatio = 0.0f;
        f32 noiseStdDev  = 0.0f;
    };

    struct TestResult
    {
        TestCase testCase;
        core::RigidTransform transform;

        std::string methodName;

        geo::ICPResult icp_result;
        bool usedESA = false;
        geo::ESAResult esa_result;
    };

    TestResult RunLeastSquaresICP(const TestCase& test, const geo::LeastSquaresICPParameters& params);

    TestResult RunSparseICPPointToPoint(const TestCase& test, const geo::SparseICPParameters& params = {});
    TestResult RunSparseICPPointToPlane(const TestCase& test, const geo::SparseICPParameters& params = {});

    TestResult RunEfficientICPPointToPlane(const TestCase& test, const geo::EfficientICPParams& params = {});

    // Prints a detailed breakdown of one alignment result to stdout.
    void LogTestResult(const TestResult& result);

    // Logs two methods side by side on the same test case.
    void LogComparison(const TestResult& r1, const TestResult& r2);
}
