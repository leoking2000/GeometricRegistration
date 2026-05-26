#include <geo/utils/logging/LogMacros.h>
#include <geo/math/Solvers.h>
#include <geo/math/Stats.h>
#include "SparseICP.h"

namespace geo
{
	// Solves for shrinkage scaling factor β in iterative Lp regularization.
	static inline f32 ComputeBeta(f32 p, f32 mu, f32 h_norm, f32 alpha_a)
	{
		f32 beta = std::clamp(alpha_a / h_norm, 0.0f, 1.0f);

		for (u8 t = 0; t < 3; t++) {
			beta = 1.0f - (p / mu) * std::pow(h_norm, p - 2.0f) * std::pow(std::max(beta, 1e-8f), p - 1.0f);
			beta = std::clamp(beta, 0.0f, 1.0f);
		}

		return beta;
	}

	// Vector-valued Lp shrink operator used in ADMM z-update.
	// Encourages sparsity of residual vectors.
	static glm::vec3 ShrinkLp(const glm::vec3& h, f32 p, f32 mu)
	{
		f32 alpha_a = std::pow((2.0f / mu) * (1.0f - p), 1.0f / (2.0f - p));
		f32 h_threshold = alpha_a + (p / mu) * std::pow(alpha_a, p - 1.0f);

		f32 h_norm = glm::length(h);
		f32 scaler = 0.0f;

		// Only shrink if residual is above threshold.
		if (h_norm > h_threshold && h_norm > 1e-12f) {
			scaler = ComputeBeta(p, mu, h_norm, alpha_a);
		}

		return scaler * h;
	}

	// Scalar version of Lp shrink operator (point-to-plane residuals).
	static f32 ShrinkLpScalar(f32 h, f32 p, f32 mu)
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
		const PointCloud3D& target, const PointCloud3D& source, 
		const INearestNeighbor& nn, SparseICPParameters params)
	{
		// Create mutable working copy of source cloud.
		// ICP progressively transforms this cloud toward the target.
		PointCloud3D src = source;

		// Basic parameter validation.
		assert(src.Size() >= 3);
		assert(target.Size() == nn.Size());
		assert(params.maxIterations >= 1);
		assert(params.tolerance > 0.0f);
		assert(params.p > 0.0f && params.p < 1.0f);
		assert(params.mu > 0.0f);
		assert(params.admmIterations >= 1);

		TimePoint startTotal = Clock::now();

		const index_t N = src.Size();

		ICPResult result;
		result.transform = RigidTransform::Identity();

		// Correspondence index buffer:
		// correspondences[i] = nearest target point for source point i.
		std::vector<index_t> correspondences(N, 0);

		// Target correspondence positions.
		std::vector<glm::vec3> targets(N, { 0.0f, 0.0f, 0.0f });

		f32 prevError = F32_MAX;  // Previous iteration error used for convergence test.

		// ADMM auxiliary variables:
		// z: sparse residual term
		// c: projected target adjustment
		// lambda: dual variable
		std::vector<glm::vec3> z(N, glm::vec3(0.0f));
		std::vector<glm::vec3> c(N, glm::vec3(0.0f));
		std::vector<glm::vec3> lambda(N, glm::vec3(0.0f));

		// Main ICP iteration loop.
		for (u32 iter = 0; iter < params.maxIterations; ++iter)
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
			for (index_t t = 0; t < N; t++)
			{
				targets[t] = target.Point(correspondences[t]);
			}

			// ---------------------------
			// 2. ADMM solve (rigid + sparse split)
			// ---------------------------

			RigidTransform localTransform = RigidTransform::Identity();
			TimePoint startSolveTime = Clock::now();

			// ADMM loop
			for (u32 admmIter = 0; admmIter < params.admmIterations; ++admmIter)
			{
				// Step 2.1: z-update (shrink)
				for (index_t i = 0; i < N; ++i)
				{
					const glm::vec3& x = src.Point(i);
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

				// rigid alignment step (SVD solve)
				localTransform = SolveRigidPointToPoint(src.GetPoints(), c);

				// Step 2.3: lambda update
				for (index_t i = 0; i < N; ++i)
				{
					const glm::vec3& x = src.Point(i);
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

			// ---------------------------
			// 3. Apply transform
			// ---------------------------

			src.Transform(localTransform);
			result.transform = RigidTransform::Compose(localTransform, result.transform);


			// ---------------------------
			// 4. Error evaluation
			// ---------------------------
			result.rmse = PointToPointRMSE(src.GetPoints(), targets);

			// ---------------------------
			// 5. Convergence check
			// ---------------------------
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

		TimePoint endTotal = Clock::now();
		result.totalTimeMs = TimeDifferenceMs(endTotal, startTotal);

		return result;
	}

	ICPResult SparseICPPointToPlane(const PointCloud3D& target, const PointCloud3D& source, 
		const INearestNeighbor& nn, SparseICPParameters params)
	{
		// Create mutable working copy of source cloud.
		// ICP progressively transforms this cloud toward the target.
		PointCloud3D src = source;

		// Basic parameter validation.
		assert(src.Size() >= 3);
		assert(target.Size() == nn.Size());
		assert(params.maxIterations >= 1);
		assert(params.tolerance > 0.0f);
		assert(params.p > 0.0f && params.p < 1.0f);
		assert(params.mu > 0.0f);
		assert(params.admmIterations >= 1);

		// Point-to-plane ICP requires target normals.
		assert(target.HasNormals());

		TimePoint startTotal = Clock::now();

		const index_t N = src.Size();

		ICPResult result;
		result.transform = RigidTransform::Identity();

		// Correspondence index buffer:
		// correspondences[i] = nearest target point for source point i.
		std::vector<index_t> correspondences(N, 0);

		std::vector<glm::vec3> targets(N, { 0.0f, 0.0f, 0.0f }); // Target correspondence positions.
		std::vector<glm::vec3> normals(N, { 1.0f, 0.0f, 0.0f }); // Target correspondence positions.

		f32 prevError = F32_MAX; // Previous iteration error used for convergence test.

		// ADMM auxiliary variables:
		// z: sparse residual term
		// c: projected target adjustment
		// lambda: dual variable
		std::vector<f32> z(N, 0.0f);
		std::vector<f32> c(N, 0.0f);
		std::vector<f32> lambda(N, 0.0f);

		// Main ICP iteration loop.
		for (u32 iter = 0; iter < params.maxIterations; ++iter)
		{
			TimePoint startTime = Clock::now();

			// ---------------------------
			// 1. Correspondence search
			// ---------------------------

			TimePoint startCorrTime = Clock::now();

			nn.QueryBatch(src.GetPoints(), correspondences);

			TimePoint endCorrTime = Clock::now();
			result.correspondenceSearchTime.AddSample(TimeDifferenceMs(endCorrTime, startCorrTime));

			// Get the target points to the buffer
			for (index_t t = 0; t < N; t++)
			{
				targets[t] = target.Point(correspondences[t]);
				normals[t] = target.Normal(correspondences[t]);
			}


			// ---------------------------
			// 2. ADMM solve (rigid + sparse split)
			// ---------------------------

			RigidTransform localTransform = RigidTransform::Identity();
			TimePoint startSolveTime = Clock::now();

			for (u32 admmIter = 0; admmIter < params.admmIterations; ++admmIter)
			{
				// Step 2.1: z-update
				for (index_t i = 0; i < N; ++i)
				{
					const glm::vec3& x = src.Point(i);
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

				// rigid alignment step
				localTransform = SolveRigidPointToPlaneShifted(src.GetPoints(), targets, normals, c);

				// Step 2.3: lambda update
				for (index_t i = 0; i < N; ++i)
				{
					const glm::vec3& x = src.Point(i);
					const glm::vec3& y = targets[i];
					const glm::vec3& n = normals[i];

					const f32 delta = glm::dot(n, localTransform.rotation * x + localTransform.translation - y);

					lambda[i] += params.mu * (delta - z[i]);
				}
			}

			TimePoint endSolveTime = Clock::now();
			result.alignmentSolveTime.AddSample(TimeDifferenceMs(endSolveTime, startSolveTime));

			// ---------------------------
			// 3. Apply transform
			// ---------------------------

			src.Transform(localTransform);
			result.transform = RigidTransform::Compose(localTransform, result.transform);

			// ---------------------------
			// 4. Error evaluation
			// ---------------------------

			result.rmse = PointToPlaneRMSE(src.GetPoints(), targets, normals);

			// ---------------------------
			// 5. Convergence check
			// ---------------------------

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

		TimePoint endTotal = Clock::now();
		result.totalTimeMs = TimeDifferenceMs(endTotal, startTotal);

		return result;
	}

}
