#include "math/Solvers.h"
#include "math/Stats.h"
#include "LeastSquaresICP.h"

namespace geo
{
	ICPResult LeastSquaresICP(
		const PointCloud3D& target, PointCloud3D& source, const INearestNeighbor& nn, LeastSquaresICPParameters params)
	{
		assert(source.Size() >= 1);
		assert(target.Size() == nn.Size());
		assert(params.maxIterations >= 1);
		assert(params.tolerance > 0.0f);
		assert((params.useNormals && target.HasNormals()) || !params.useNormals);

		index_t numberOfPoints = source.Size();

		ICPResult result;
		result.transform = RigidTransform::Identity();

		f64 prevError = F32_MAX;

		std::vector<index_t> correspondences(numberOfPoints, 0);

		std::vector<glm::vec3> targets(numberOfPoints, { 0.0f, 0.0f, 0.0f });
		std::vector<glm::vec3> normals(numberOfPoints, { 1.0f, 0.0f, 0.0f });

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
				if (params.useNormals && target.HasNormals()) {
					normals[t] = target.Normal(correspondences[t]);
				}
			}
			
			// Solve System
			TimePoint startSolveTime = Clock::now();

			RigidTransform localTransform = RigidTransform::Identity();

			if(params.useNormals && target.HasNormals())
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
			result.rmse = RMSE(numberOfPoints, [&](index_t i) -> f32 {
				f32 residual2 = 0.0f;

				if (params.useNormals && target.HasNormals())
				{
					glm::vec3 diff = source.Point(i) - targets[i];
					residual2 = glm::dot(diff, normals[i]) * glm::dot(diff, normals[i]);
				}
				else
				{
					glm::vec3 diff = source.Point(i) - targets[i];
					residual2 = glm::dot(diff, diff);
				}

				return residual2;
			});

			result.iterations = iter + 1;

			// converged check
			if (std::abs(prevError - result.rmse) < params.tolerance)
			{
				result.converged = true;
				break;
			}

			prevError = result.rmse;

			TimePoint endTime = Clock::now();

			result.totalIterationTime.AddSample(TimeDifferenceMs(endTime, startTime));
		}

		return result;
	}

}
