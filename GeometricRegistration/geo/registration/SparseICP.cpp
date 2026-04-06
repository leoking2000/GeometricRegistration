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

	static inline glm::vec3 ShrinkLp(const glm::vec3& h, f32 p, f32 mu)
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

	ICPResult SparseICP(
		const PointCloud3D& target, PointCloud3D& source, const INearestNeighbor& nn, SparseICPParameters params)
	{
		assert(source.Size() >= 1);
		assert(target.Size() == nn.Size());
		assert(params.maxIterations >= 1);
		assert(params.tolerance > 0.0f);
		assert((params.useNormals && target.HasNormals()) || !params.useNormals);
		assert(params.p > 0.0f && params.p < 1.0f);
		assert(params.mu > 0.0f);
		assert(params.admmIterations >= 1);

		const index_t N = source.Size();

		ICPResult result;
		result.transform = { glm::mat3(1.0f), glm::vec3(0.0f) };

		std::vector<index_t> correspondences(N, 0);

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

			RigidTransform localTransform = { glm::mat3(1.0f), glm::vec3(0.0f) };

			// Reset ADMM variables per ICP iteration
			//std::fill(z.begin(), z.end(), glm::vec3(0.0f));
			//std::fill(lambda.begin(), lambda.end(), glm::vec3(0.0f));

			TimePoint startSolveTime = Clock::now();

			// Step 2: run ADMM to solve for RigidTransform

			for (u32 admmIter = 0; admmIter < params.admmIterations; ++admmIter)
			{
				// Step 2.1: z-update (shrink)
				for (index_t i = 0; i < N; ++i)
				{
					const glm::vec3& x = source.Point(i);
					const glm::vec3& y = target.Point(correspondences[i]);

					glm::vec3 hi = localTransform.rotation * x
						+ localTransform.translation
						- y
						+ lambda[i] / params.mu;

					z[i] = ShrinkLp(hi, params.p, params.mu);
				}

				// Step 2.2: rigid update
				for (index_t i = 0; i < N; ++i)
				{
					const glm::vec3& y = target.Point(correspondences[i]);
					c[i] = y + z[i] - lambda[i] / params.mu;
				}

				localTransform = SolveRigidPointToPoint(source.GetPoints(), c);

				// Step 2.3: lambda update
				for (index_t i = 0; i < N; ++i)
				{
					const glm::vec3& x = source.Point(i);
					const glm::vec3& y = target.Point(correspondences[i]);

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


			// Step 4: compute sparse-style RMS
			result.rmse = RMSE(N, [&](index_t i) -> f32 {
				f32 residual2 = 0.0f;

				if (params.useNormals && target.HasNormals())
				{
					glm::vec3 diff = source.Point(i) - target.Point(correspondences[i]);
					residual2 = glm::dot(diff, target.Normal(correspondences[i])) * glm::dot(diff, target.Normal(correspondences[i]));
				}
				else
				{
					glm::vec3 diff = source.Point(i) - target.Point(correspondences[i]);
					residual2 = glm::dot(diff, diff);
				}

				return residual2;
				});

			result.iterations = iter + 1;

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
