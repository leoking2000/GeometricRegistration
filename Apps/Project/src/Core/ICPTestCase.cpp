#include "ICPTestCase.h"

#include <glm/gtc/matrix_transform.hpp>

namespace test
{
    ICPTestCase KnowedTransform(const geo::PointCloud3D& target, const geo::RigidTransform& T, const std::string& name)
    {
        geo::PointCloud3D source = target;
        source.Transform(T);

        return {
            name,
            target,
            source,
            T
        };
    }

}