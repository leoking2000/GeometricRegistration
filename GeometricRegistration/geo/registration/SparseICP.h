#pragma once
#include "geometry/PointCloud3D.h"
#include "spatial/INearestNeighbor.h"
#include "ICPResult.h"

namespace geo
{
    struct SparseICPParameters
    {
        u32 maxIterations = 500;
        f32 tolerance = 1e-5f;
        f32 p = 0.4f;
        f32 mu = 10;
        u32 admmIterations = 10;
    };

    ICPResult SparseICPPointToPoint(
        const PointCloud3D& target, PointCloud3D& source, const INearestNeighbor& nn, SparseICPParameters params = {});

    ICPResult SparseICPPointToPlane(
        const PointCloud3D& target, PointCloud3D& source, const INearestNeighbor& nn, SparseICPParameters params = {});
}
