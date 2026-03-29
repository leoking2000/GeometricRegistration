#include "math/SVD.h"
#include "SparseICP.h"

namespace geo
{
	using Clock = std::chrono::steady_clock;
	using TimePoint = Clock::time_point;

	// Compute a - b in milliseconds
	static f64 TimeDifference_ms(TimePoint end, TimePoint start)
	{
		return std::chrono::duration<f64, std::milli>(end - start).count();
	}

	static inline RigidTransform SolveRigid_PointToPoint_ToTargets(
		const PointCloud3D& source,
		const std::vector<glm::vec3>& targets)
	{
		assert(source.Size() == targets.size());
		assert(source.Size() > 0);

		const index_t N = source.Size();

		glm::vec3 centroidSrc(0.0f);
		glm::vec3 centroidTgt(0.0f);

		for (index_t i = 0; i < N; ++i)
		{
			centroidSrc += source[i];
			centroidTgt += targets[i];
		}

		centroidSrc /= static_cast<f32>(N);
		centroidTgt /= static_cast<f32>(N);

		glm::mat3 H(0.0f);
		for (index_t i = 0; i < N; ++i)
		{
			glm::vec3 p = source[i] - centroidSrc;
			glm::vec3 q = targets[i] - centroidTgt;
			H += glm::outerProduct(p, q);
		}

		SVDResult svd = SVD(H);

		glm::mat3 Ut = glm::transpose(svd.U);
		glm::mat3 R = svd.V * Ut;

		if (glm::determinant(R) < 0.0f)
		{
			svd.V[2] *= -1.0f;
			R = svd.V * Ut;
		}

		glm::vec3 t = centroidTgt - R * centroidSrc;
		return { R, t };
	}

	static inline f32 ComputeBeta(f32 p, f32 mu, f32 h_norm, f32 alpha_a)
	{
		f32 beta = std::clamp(alpha_a / h_norm, 0.0f, 1.0f);

		for (u8 t = 0; t < 3; t++) {
			beta = 1.0 - (p / mu) * std::pow(h_norm, p - 2.0) * std::pow(std::max(beta, 1e-8f), p - 1.0);
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
		const PointCloud3D& target, 
		PointCloud3D& source, 
		const INearestNeighbor& nn, 
		u32 maxIterations, f32 p, f32 mu, u32 admmIterations, f32 tolerance)
	{
		assert(!target.Empty());
		assert(!source.Empty());
		assert(target.Size() == nn.Size());
		assert(maxIterations >= 1);
		assert(tolerance > 0.0f);
		assert(p > 0.0f && p < 1.0f);
		assert(mu > 0.0f);
		assert(admmIterations >= 1);

		const index_t N = source.Size();

		ICPResult result;
		result.transform = { glm::mat3(1.0f), glm::vec3(0.0f) };

		std::vector<index_t> correspondences(N, 0);

		f32 prevError = F32_MAX;

		// ADMM variables
		std::vector<glm::vec3> z(N, glm::vec3(0.0f));
		std::vector<glm::vec3> h(N, glm::vec3(0.0f));
		std::vector<glm::vec3> c(N, glm::vec3(0.0f));
		std::vector<glm::vec3> lambda(N, glm::vec3(0.0f));

		TimePoint startTime = Clock::now();

		for (u32 iter = 0; iter < maxIterations; ++iter)
		{
			// Step 1: correspondences
			TimePoint startCorrTime = Clock::now();
			nn.QueryBatch(source.GetStorage(), correspondences);
			TimePoint endCorrTime = Clock::now();
			result.avgCorrespondenceTime_ms += TimeDifference_ms(endCorrTime, startCorrTime);

			RigidTransform localTransform = { glm::mat3(1.0f), glm::vec3(0.0f) };

			// Reset ADMM variables per ICP iteration
			std::fill(z.begin(), z.end(), glm::vec3(0.0f));
			std::fill(lambda.begin(), lambda.end(), glm::vec3(0.0f));

			TimePoint startSolveTime = Clock::now();

			// Step 2: run ADMM to solve for RigidTransform

			for (u32 admmIter = 0; admmIter < admmIterations; ++admmIter)
			{
				// Step 2.1: z-update (shrink)
				for (index_t i = 0; i < N; ++i)
				{
					const glm::vec3& x = source[i];
					const glm::vec3& y = target[correspondences[i]];

					h[i] = localTransform.rotation * x
						+ localTransform.translation
						- y
						+ lambda[i] / mu;

					z[i] = ShrinkLp(h[i], p, mu);
				}

				// Step 2.2: rigid update
				for (index_t i = 0; i < N; ++i)
				{
					const glm::vec3& y = target[correspondences[i]];
					c[i] = y + z[i] - lambda[i] / mu;
				}

				localTransform = SolveRigid_PointToPoint_ToTargets(source, c);

				// Step 2.3: lambda update
				for (index_t i = 0; i < N; ++i)
				{
					const glm::vec3& x = source[i];
					const glm::vec3& y = target[correspondences[i]];

					glm::vec3 delta =
						localTransform.rotation * x
						+ localTransform.translation
						- y
						- z[i];

					lambda[i] += mu * delta;
				}
			}

			TimePoint endSolveTime = Clock::now();
			result.avgSolverTime_ms += TimeDifference_ms(endSolveTime, startSolveTime);

			// Step 3: apply transform
			source.Transform(localTransform);

			result.transform.translation = localTransform.rotation * result.transform.translation + localTransform.translation;
			result.transform.rotation = localTransform.rotation * result.transform.rotation;

			// Step 4: compute sparse-style RMS
			f64 error = 0.0;
			for (index_t i = 0; i < N; ++i)
			{
				glm::vec3 d = source[i] - target[correspondences[i]];
				error += glm::dot(d, d);
			}

			result.rms = static_cast<f32>(std::sqrt(error / static_cast<f64>(N)));
			result.iterations = iter + 1;

			if (std::abs(prevError - result.rms) < tolerance)
			{
				result.converged = true;
				break;
			}

			prevError = result.rms;
		}

		TimePoint endTime = Clock::now();
		result.totalElapsed_ms = TimeDifference_ms(endTime, startTime);

		if (result.iterations > 0)
		{
			result.avgCorrespondenceTime_ms /= result.iterations;
			result.avgSolverTime_ms /= result.iterations;
		}

		return result;
	}

}
