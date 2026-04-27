#include <geo/utils/logging/LogMacros.h>
#include "math/Solvers.h"
#include "math/Stats.h"
#include "SparseICP.h"

namespace geo
{
	static inline f32 ComputeBeta(f32 p, f32 mu, f32 h_norm, f32 alpha_a)
	{
		f32 beta = std::clamp(alpha_a / h_norm, 0.0f, 1.0f);

		for (u8 t = 0; t < 3; t++) {
			beta = 1.0f - (p / mu) * std::pow(h_norm, p - 2.0f) * std::pow(std::max(beta, 1e-8f), p - 1.0f);
			beta = std::clamp(beta, 0.0f, 1.0f);
		}

		return beta;
	}

	glm::vec3 ShrinkLp(const glm::vec3& h, f32 p, f32 mu)
	{
		f32 alpha_a = std::pow((2.0f / mu) * (1.0f - p), 1.0f / (2.0f - p));
		f32 h_threshold = alpha_a + (p / mu) * std::pow(alpha_a, p - 1.0f);

		f32 h_norm = glm::length(h);
		f32 scaler = 0.0f;
		if (h_norm > h_threshold && h_norm > 1e-12f) {
			scaler = ComputeBeta(p, mu, h_norm, alpha_a);
		}

		return scaler * h;
	}

	f32 ShrinkLpScalar(f32 h, f32 p, f32 mu)
	{
		f32 alpha_a = std::pow((2.0f / mu) * (1.0f - p), 1.0f / (2.0f - p));
		f32 h_threshold = alpha_a + (p / mu) * std::pow(alpha_a, p - 1.0f);

		f32 h_norm = glm::abs(h);
		f32 scaler = 0.0f;
		if (h_norm > h_threshold && h_norm > 1e-12f) {
			scaler = ComputeBeta(p, mu, h_norm, alpha_a);
		}

		return scaler * h;
	}

	ICPResult SparseICPPointToPoint(
		const PointCloud3D& target, PointCloud3D& source, const INearestNeighbor& nn, SparseICPParameters params)
	{
		assert(source.Size() >= 3);
		assert(target.Size() == nn.Size());
		assert(params.maxIterations >= 1);
		assert(params.tolerance > 0.0f);
		assert(params.p > 0.0f && params.p < 1.0f);
		assert(params.mu > 0.0f);
		assert(params.admmIterations >= 1);

		const index_t N = source.Size();

		ICPResult result;
		result.transform = RigidTransform::Identity();

		std::vector<index_t> correspondences(N, 0);

		std::vector<glm::vec3> targets(N, { 0.0f, 0.0f, 0.0f });

		f32 prevError = F32_MAX;

		// ADMM variables
		std::vector<glm::vec3> z(N, glm::vec3(0.0f));
		std::vector<glm::vec3> c(N, glm::vec3(0.0f));
		std::vector<glm::vec3> lambda(N, glm::vec3(0.0f));

		for (u32 iter = 0; iter < params.maxIterations; ++iter)
		{
			TimePoint startTime = Clock::now();

			// Step 1: correspondences

			TimePoint startCorrTime = Clock::now();
			nn.QueryBatch(source.GetPoints(), correspondences);
			TimePoint endCorrTime = Clock::now();

			result.correspondenceSearchTime.AddSample(TimeDifferenceMs(endCorrTime, startCorrTime));

			// Get the target points to the buffer
			for (index_t t = 0; t < N; t++)
			{
				targets[t] = target.Point(correspondences[t]);
			}

			RigidTransform localTransform = RigidTransform::Identity();

			TimePoint startSolveTime = Clock::now();

			// Step 2: run ADMM to solve for RigidTransform

			for (u32 admmIter = 0; admmIter < params.admmIterations; ++admmIter)
			{
				// Step 2.1: z-update (shrink)
				for (index_t i = 0; i < N; ++i)
				{
					const glm::vec3& x = source.Point(i);
					const glm::vec3& y = targets[i];

					glm::vec3 hi = localTransform.rotation * x
						+ localTransform.translation
						- y
						+ lambda[i] / params.mu;

					z[i] = ShrinkLp(hi, params.p, params.mu);
				}

				// Step 2.2: rigid update
				for (index_t i = 0; i < N; ++i)
				{
					const glm::vec3& y = targets[i];
					c[i] = y + z[i] - lambda[i] / params.mu;
				}

				localTransform = SolveRigidPointToPoint(source.GetPoints(), c);

				// Step 2.3: lambda update
				for (index_t i = 0; i < N; ++i)
				{
					const glm::vec3& x = source.Point(i);
					const glm::vec3& y = targets[i];

					glm::vec3 delta =
						localTransform.rotation * x
						+ localTransform.translation
						- y
						- z[i];

					lambda[i] += params.mu * delta;
				}
			}

			TimePoint endSolveTime = Clock::now();

			result.alignmentSolveTime.AddSample(TimeDifferenceMs(endSolveTime, startSolveTime));

			// Step 3: apply transform
			source.Transform(localTransform);

			result.transform = RigidTransform::Compose(localTransform, result.transform);


			// Step 4: compute RMSE
			result.rmse = PointToPointRMSE(source.GetPoints(), targets);

			// Step 5: converged check
			const f32 transNorm = glm::length(localTransform.translation);
			const f32 rotAngle = RotationAngle(localTransform.rotation);

			const bool smallMotion = (transNorm < params.transTolerance) && (rotAngle < params.rotTolerance);
			const bool smallErrorChange = std::abs(prevError - result.rmse) < params.tolerance;

			prevError = result.rmse;
			result.iterations = iter + 1;

			TimePoint endTime = Clock::now();
			result.totalIterationTime.AddSample(TimeDifferenceMs(endTime, startTime));

			GEOLOGDEBUG("iter: " << iter + 1
				<< " rmse: " << result.rmse
				<< " trans: " << transNorm
				<< " rot: " << rotAngle << "\n");

			if (smallMotion && smallErrorChange)
			{
				result.converged = true;
				break;
			}
		}

		return result;
	}

	ICPResult SparseICPPointToPlane(const PointCloud3D& target, PointCloud3D& source, const INearestNeighbor& nn, SparseICPParameters params)
	{
		assert(source.Size() >= 3);
		assert(target.Size() == nn.Size());
		assert(params.maxIterations >= 1);
		assert(params.tolerance > 0.0f);
		assert(params.p > 0.0f && params.p < 1.0f);
		assert(params.mu > 0.0f);
		assert(params.admmIterations >= 1);
		assert(target.HasNormals());

		const index_t N = source.Size();

		ICPResult result;
		result.transform = RigidTransform::Identity();

		std::vector<index_t> correspondences(N, 0);

		std::vector<glm::vec3> targets(N, { 0.0f, 0.0f, 0.0f });
		std::vector<glm::vec3> normals(N, { 1.0f, 0.0f, 0.0f });

		f32 prevError = F32_MAX;

		// ADMM variables
		std::vector<f32> z(N, 0.0f);
		std::vector<f32> c(N, 0.0f);
		std::vector<f32> lambda(N, 0.0f);

		for (u32 iter = 0; iter < params.maxIterations; ++iter)
		{
			TimePoint startTime = Clock::now();

			// Step 1: correspondences

			TimePoint startCorrTime = Clock::now();
			nn.QueryBatch(source.GetPoints(), correspondences);
			TimePoint endCorrTime = Clock::now();

			result.correspondenceSearchTime.AddSample(TimeDifferenceMs(endCorrTime, startCorrTime));

			// Get the target points to the buffer
			for (index_t t = 0; t < N; t++)
			{
				targets[t] = target.Point(correspondences[t]);
				normals[t] = target.Normal(correspondences[t]);
			}

			RigidTransform localTransform = RigidTransform::Identity();

			TimePoint startSolveTime = Clock::now();

			// Step 2: run ADMM to solve for RigidTransform

			for (u32 admmIter = 0; admmIter < params.admmIterations; ++admmIter)
			{
				// Step 2.1: z-update
				for (index_t i = 0; i < N; ++i)
				{
					const glm::vec3& x = source.Point(i);
					const glm::vec3& y = targets[i];
					const glm::vec3& n = normals[i];

					const f32 delta = glm::dot(n, localTransform.rotation * x + localTransform.translation - y);
					const f32 h = delta + lambda[i] / params.mu;

					z[i] = ShrinkLpScalar(h, params.p, params.mu);
				}

				// Step 2.2: rigid update
				for (index_t i = 0; i < N; ++i)
				{
					c[i] = z[i] - lambda[i] / params.mu;
				}

				localTransform = SolveRigidPointToPlaneShifted(source.GetPoints(), targets, normals, c);

				// Step 2.3: lambda update
				for (index_t i = 0; i < N; ++i)
				{
					const glm::vec3& x = source.Point(i);
					const glm::vec3& y = targets[i];
					const glm::vec3& n = normals[i];

					const f32 delta = glm::dot(n, localTransform.rotation * x + localTransform.translation - y);

					lambda[i] += params.mu * (delta - z[i]);
				}
			}

			TimePoint endSolveTime = Clock::now();

			result.alignmentSolveTime.AddSample(TimeDifferenceMs(endSolveTime, startSolveTime));

			// Step 3: apply transform
			source.Transform(localTransform);

			result.transform = RigidTransform::Compose(localTransform, result.transform);


			// Step 4: compute RMSE on current correspondences
			//result.rmse = PointToPointRMSE(source.GetPoints(), targets);
			result.rmse = PointToPlaneRMSE(source.GetPoints(), targets, normals);

			result.iterations = iter + 1;

			TimePoint endTime = Clock::now();
			result.totalIterationTime.AddSample(TimeDifferenceMs(endTime, startTime));

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
