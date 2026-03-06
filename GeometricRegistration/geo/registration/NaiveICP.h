#pragma once
#include "geometry/PointCloud3D.h"
#include "spatial/INearestNeighbor.h"
#include "ICPResult.h"


namespace geo
{
	// Least squares
	ICPResult NaiveICP(const INearestNeighbor& target, PointCloud3D& source, int maxIterations, float tolerance = 1e-5f);
}
