#pragma once
#include "geometry/PointCloud3D.h"
#include "spatial/INearestNeighbor.h"
#include "ICPResult.h"

namespace geo
{
	// Least squares
	ICPResult NaiveICP(
		const PointCloud3D& tatget, 
		PointCloud3D& source, 
		const INearestNeighbor& nn, 
		u32 maxIterations, f32 tolerance = 1e-5f, bool useNormals = true);
}
