#pragma once
#include <geo/geometry/PointCloud3D.h>
#include <geo/spatial/INearestNeighbor.h>
#include "ICPResult.h"

namespace geo
{
	// Parameters controlling least-squares ICP convergence and behavior.
	struct LeastSquaresICPParameters
	{
		// Maximum number of ICP iterations before forced termination.
		u32 maxIterations = 500;

		// RMSE convergence threshold.
		// Optimization stops when the RMSE improvement falls below this value.
		f32 tolerance = 1e-5f;

		// Translation convergence threshold.
		// Measured in world-space units.
		f32 transTolerance = 1e-5f;

		// Rotation convergence threshold in radians.
		// Used to detect very small rotational updates.
		f32 rotTolerance = 1e-4f;

		// If true: uses point-to-plane alignment (requires target normals).
		// If false: uses point-to-point alignment.
		bool useNormals = false;
	};

	// Performs least-squares ICP registration.
	//
	// target:
	//   Fixed/reference point cloud.
	//
	// source:
	//   Moving point cloud to be aligned onto target.
	//
	// nn:
	//   Nearest-neighbor structure built over target points.
	//
	// params:
	//   ICP solver configuration.
	//
	// Returns:
	//   Final transform, RMSE, convergence state, and timing statistics.
	ICPResult LeastSquaresICP(const PointCloud3D& target, const PointCloud3D& source,
		const INearestNeighbor& nn, const LeastSquaresICPParameters& params = {});
}
