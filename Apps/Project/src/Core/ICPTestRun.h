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

        geo::u32 sourceCount = 0;
        geo::u32 targetCount = 0;
    };

    void PrintResult(const ICPTestResult& r);

    ICPTestResult RunLeastSquaresICP(const ICPTestCase& test, const geo::LeastSquaresICPParameters& params = {});
    ICPTestResult RunSparseICPPointToPoint(const ICPTestCase& test, const geo::SparseICPParameters& params = {});
    ICPTestResult RunSparseICPPointToPlane(const ICPTestCase& test, const geo::SparseICPParameters& params = {});
}
