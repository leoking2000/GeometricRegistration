#include <geo/utils/logging/LogMacros.h>
#include <geo/math/Solvers.h>
#include "LeastSquaresICP.h"

namespace geo
{
	ICPResult LeastSquaresICP(
		const PointCloud3D& target, const PointCloud3D& source, 
		const INearestNeighbor& nn, const LeastSquaresICPParameters& params)
	{
		// Basic parameter validation.
		assert(source.Size() >= 3);
		assert(target.Size() == nn.Size());
		assert(params.maxIterations >= 1);
		assert(params.tolerance > 0.0f);

		// Point-to-plane ICP requires target normals.
		const bool usePointToPlane = params.useNormals && target.HasNormals(); // Select ICP variant.
		assert((params.useNormals && target.HasNormals()) || !params.useNormals);

		GEOLOGINFO("LeastSquaresICP started");
		GEOLOGINFO("ICP mode: " << (usePointToPlane ? "PointToPlane" : "PointToPoint"));

		GEOLOGDEBUG("Target points: " << target.Size()
			<< " | Source points: " << source.Size()
			<< " | Max iterations: " << params.maxIterations);

		// Create mutable working copy of source cloud.
		// ICP progressively transforms this cloud toward the target.
		PointCloud3D src = source;
		TimePoint startTotal = Clock::now();

		const index_t numberOfPoints = src.Size();

		ICPResult result;
		result.transform = RigidTransform::Identity();

		f32 prevError = F32_MAX; // Previous iteration error used for convergence test.

		// Correspondence index buffer:
		// correspondences[i] = nearest target point for source point i.
		std::vector<index_t> correspondences(numberOfPoints, 0);

		// Target correspondence positions.
		std::vector<glm::vec3> targets(numberOfPoints, { 0.0f, 0.0f, 0.0f });
		std::vector<glm::vec3> normals; // Target normals used only for point-to-plane ICP.

		if (usePointToPlane) {
			normals.resize(numberOfPoints, { 1.0f, 0.0f, 0.0f });
		}

		// Main ICP iteration loop.
		for (u32 iter = 0; iter < params.maxIterations; iter++)
		{
			TimePoint startTime = Clock::now();

			// ------------------------------------------------------------
			// 1. Find nearest-neighbor correspondences
			// ------------------------------------------------------------

			TimePoint startCorrTime = Clock::now();

			nn.QueryBatch(src.GetPoints(), correspondences);

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

			// ------------------------------------------------------------
			// 2. Solve rigid alignment
			// ------------------------------------------------------------

			TimePoint startSolveTime = Clock::now();

			RigidTransform localTransform = RigidTransform::Identity();
			if(usePointToPlane)
			{
				// Point-to-plane least squares solve.
				localTransform = SolveRigidPointToPlane(src.GetPoints(), targets, normals);
			}
			else
			{
				// Classic point-to-point SVD solve.
				localTransform = SolveRigidPointToPoint(src.GetPoints(), targets);
			}

			TimePoint endSolveTime = Clock::now();
			result.alignmentSolveTime.AddSample(TimeDifferenceMs(endSolveTime, startSolveTime));


			// ------------------------------------------------------------
			// 3. Apply incremental transform
			// ------------------------------------------------------------

			src.Transform(localTransform);
			// Accumulate total transform:
			// new_total = local * previous_total
			result.transform = RigidTransform::Compose(localTransform, result.transform);

			// ------------------------------------------------------------
			// 4. Compute current registration error
			// ------------------------------------------------------------

			if (usePointToPlane)
			{
				result.rmse = PointToPlaneRMSE(src.GetPoints(), targets, normals);
			}
			else
			{
				result.rmse = PointToPointRMSE(src.GetPoints(), targets);
			}

			// ------------------------------------------------------------
			// 5. Convergence checks
			// ------------------------------------------------------------
			
			// Magnitude of incremental translation update.
			const f32 transNorm = glm::length(localTransform.translation);
			// Magnitude of incremental rotational update.
			const f32 rotAngle = RotationAngle(localTransform.rotation);

			// Converged if transform update is very small.
			const bool smallMotion = (transNorm < params.transTolerance) && (rotAngle < params.rotTolerance);
			// Converged if RMSE improvement becomes negligible.
			const bool smallErrorChange = std::abs(prevError - result.rmse) < params.tolerance;

			prevError = result.rmse;
			result.iterations = iter + 1;

			TimePoint endTime = Clock::now();
			result.totalIterationTime.AddSample(TimeDifferenceMs(endTime, startTime));

			// VERBOSE iteration statistics.
			GEOLOGVERBOSE(
				"[ICP] iter=" << (iter + 1)
				<< " rmse=" << result.rmse
				<< " dRMSE=" << (prevError - result.rmse)
				<< " trans=" << transNorm
				<< " rot=" << rotAngle
			);

			if (smallMotion)
			{
				GEOLOGDEBUG("[ICP] small motion detected: trans=" << transNorm << " rot=" << rotAngle);
			}

			if (smallErrorChange)
			{
				GEOLOGDEBUG("[ICP] small error change detected: dRMSE=" << std::abs(prevError - result.rmse));
			}

			// Stop once both geometric motion or error change are small.
			if (smallMotion || smallErrorChange)
			{
				result.converged = true;
				break;
			}
		}

		TimePoint endTotal = Clock::now();
		result.totalTimeMs = TimeDifferenceMs(endTotal, startTotal); // Final total ICP runtime in milliseconds.

		GEOLOGINFO("LeastSquaresICP finished"
			<< " | iterations=" << result.iterations
			<< " | rmse=" << result.rmse
			<< " | time_ms=" << result.totalTimeMs
			<< " | converged=" << result.converged);

		return result;
	}

}
