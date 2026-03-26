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
        f32 tolerance = 1e-5f,
        f32 p = 1.0f,
        f32 epsilon = 1e-6f);
}
