#include "ICPTestCase.h"

namespace test
{
    struct ICPTestResult
    {
        std::string methodName;
        std::string testName;

        geo::ICPResult result;

        geo::f32 rotationError = 0.0f;     // angle difference
        geo::f32 translationError = 0.0f;  // L2 error
    };

    ICPTestResult RunLeastSquares(const ICPTestCase& test, const geo::LeastSquaresICPParameters& params = {});
    void PrintResult(const ICPTestResult& r);
}
