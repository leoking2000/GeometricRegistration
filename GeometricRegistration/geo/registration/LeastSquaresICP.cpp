#include "math/Solvers.h"
#include "math/Stats.h"
#include "LeastSquaresICP.h"

namespace geo
{
	ICPResult LeastSquaresICP(
		const PointCloud3D& target, PointCloud3D& source, const INearestNeighbor& nn, const LeastSquaresICPParameters& params)
	{
		assert(source.Size() >= 3);
		assert(target.Size() == nn.Size());
		assert(params.maxIterations >= 1);
		assert(params.tolerance > 0.0f);
		assert((params.useNormals && target.HasNormals()) || !params.useNormals);

		const bool usePointToPlane = params.useNormals && target.HasNormals();

		const index_t numberOfPoints = source.Size();

		ICPResult result;
		result.transform = RigidTransform::Identity();

		f64 prevError = F32_MAX;

		std::vector<index_t> correspondences(numberOfPoints, 0);

		std::vector<glm::vec3> targets(numberOfPoints, { 0.0f, 0.0f, 0.0f });
		std::vector<glm::vec3> normals;

		if (usePointToPlane) {
			normals.resize(numberOfPoints, { 1.0f, 0.0f, 0.0f });
		}

		for (u32 iter = 0; iter < params.maxIterations; iter++)
		{
			TimePoint startTime = Clock::now();

			// Find correspondences 
			TimePoint startCorrTime = Clock::now();

			nn.QueryBatch(source.GetPoints(), correspondences);

			TimePoint endCorrTime = Clock::now();
			result.correspondenceSearchTime.AddSample(TimeDifferenceMs(endCorrTime, startCorrTime));

			// Get the target points to the buffer
			for (index_t t = 0; t < numberOfPoints; t++)
			{
				targets[t] = target.Point(correspondences[t]);
				if (usePointToPlane) {
					normals[t] = target.Normal(correspondences[t]);
				}
			}
			
			// Solve System
			TimePoint startSolveTime = Clock::now();

			RigidTransform localTransform = RigidTransform::Identity();

			if(usePointToPlane)
			{
				localTransform = SolveRigidPointToPlane(source.GetPoints(), targets, normals);
			}
			else
			{
				localTransform = SolveRigidPointToPoint(source.GetPoints(), targets);
			}

			TimePoint endSolveTime = Clock::now();
			result.alignmentSolveTime.AddSample(TimeDifferenceMs(endSolveTime, startSolveTime));

			// Apply transform
			source.Transform(localTransform);

			result.transform = RigidTransform::Compose(localTransform, result.transform);

			// Compute RMS
			if (usePointToPlane)
			{
				result.rmse = PointToPlaneRMSE(source.GetPoints(), targets, normals);
			}
			else
			{
				result.rmse = PointToPointRMSE(source.GetPoints(), targets);
			}

			result.iterations = iter + 1;

			TimePoint endTime = Clock::now();
			result.totalIterationTime.AddSample(TimeDifferenceMs(endTime, startTime));

			// converged check
			if (std::abs(prevError - result.rmse) < params.tolerance)
			{
				result.converged = true;
				break;
			}

			prevError = result.rmse;
		}

		return result;
	}

}
