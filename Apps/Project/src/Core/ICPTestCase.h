#include <string>
#include <functional>
#include <geo/GeometricRegistration.h>

namespace test
{
    struct ICPTestCase
    {
        std::string name;

        geo::PointCloud3D target;
        geo::PointCloud3D source;

        geo::RigidTransform groundTruth;
    };

    ICPTestCase KnowedTransform(const geo::PointCloud3D& target, const geo::RigidTransform& T, const std::string& name = "KnowedTransform");

    ICPTestCase PartialOverlap(
        const geo::PointCloud3D& target, const geo::RigidTransform& T, 
        std::function<bool(const glm::vec3&)> keep, const std::string& name = "PartialOverlap");

    ICPTestCase WithOutliers(const geo::PointCloud3D& target, const geo::RigidTransform& T, 
        geo::u32 outlierCount, geo::f32 range, const std::string& name = "WithOutliers", geo::u32 seed = 0);
}