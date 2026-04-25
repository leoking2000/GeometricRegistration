#pragma once
#include "geometry/PointCloud3D.h"
#include "spatial/INearestNeighbor.h"
#include "ICPResult.h"

namespace geo
{
	struct LeastSquaresICPParameters
	{
		u32 maxIterations = 500;
		f32 tolerance = 1e-5f; // RMSE tolerance
		f32 transTolerance = 1e-5f;
		f32 rotTolerance = 1e-4f; // radians
		bool useNormals = false;
	};

	f32 RotationAngle(const glm::mat3& R);

	// Least squares ICP
	ICPResult LeastSquaresICP(
		const PointCloud3D& target, PointCloud3D& source, const INearestNeighbor& nn, const LeastSquaresICPParameters& params = {});
}
