#include <string>
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

}