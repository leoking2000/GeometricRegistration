#pragma once
#include "geometry/PointCloud3D.h"
#include "spatial/INearestNeighbor.h"
#include "ICPResult.h"

namespace geo
{
	struct LeastSquaresICPParameters
	{
		u32 maxIterations = 500;
		f32 tolerance = 1e-5f;
		bool useNormals = false;
	};

	// Least squares ICP
	ICPResult LeastSquaresICP(
		const PointCloud3D& target, PointCloud3D& source, const INearestNeighbor& nn, LeastSquaresICPParameters params = {});
}
