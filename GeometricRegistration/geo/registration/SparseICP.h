#pragma once
#include "geometry/PointCloud3D.h"
#include "spatial/INearestNeighbor.h"
#include "ICPResult.h"

namespace geo
{
    ICPResult SparseICP(
        const PointCloud3D& target,
        PointCloud3D& source,
        const INearestNeighbor& nn,
        u32 maxIterations,
        f32 p = 0.9f,
        f32 mu = 10.0f,
        u32 admmIterations = 10,
        f32 tolerance = 1e-5f);
}
