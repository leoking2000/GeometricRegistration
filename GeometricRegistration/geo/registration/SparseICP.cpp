#include "math/SVD.h"
#include "SparseICP.h"

#define IRLS

namespace geo
{
	using Clock = std::chrono::steady_clock;
	using TimePoint = Clock::time_point;

	// Compute a - b in milliseconds
	static f64 TimeDifference_ms(TimePoint end, TimePoint start)
	{
		return std::chrono::duration<f64, std::milli>(end - start).count();
	}

	static inline RigidTransform WeightedPointToPoint(
		const PointCloud3D& target,
		const PointCloud3D& source,
		const std::vector<index_t>& correspondences,
		const std::vector<f32>& weights)
	{
		const index_t N = (index_t)source.Size();
		assert(correspondences.size() == N);
		assert(weights.size() == N);

		f32 sumW = 0.0;
		glm::vec3 centroidSrc(0.0);
		glm::vec3 centroidTgt(0.0);

		for (index_t i = 0; i < N; i++)
		{
			f32 w = weights[i];
			sumW += w;
			centroidSrc += w * source[i];
			centroidTgt += w * target[correspondences[i]];
		}

		if (sumW <= 0.0)
		{
			return { glm::mat3(1.0f), glm::vec3(0.0f) };
		}

		centroidSrc /= sumW;
		centroidTgt /= sumW;

		glm::mat3 H(0.0);

		for (index_t i = 0; i < N; ++i)
		{
			const f64 w = (f64)weights[i];

			glm::dvec3 p = source[i] - centroidSrc;
			glm::dvec3 q = target[correspondences[i]] - centroidTgt;

			H += w * glm::outerProduct(p, q);
		}

		SVDResult svd = SVD(H);
		glm::mat3 Ut = glm::transpose(svd.U);
		glm::mat3 R = svd.V * Ut;

		if (glm::determinant(R) < 0.0f)
		{
			svd.V[2] *= -1.0f;
			R = svd.V * Ut;
		}

		glm::vec3 t = glm::vec3(centroidTgt) - R * glm::vec3(centroidSrc);

		return { R, t };
	}


	ICPResult SparseICP(
		const PointCloud3D& target, 
		PointCloud3D& source, 
		const INearestNeighbor& nn, 
		u32 maxIterations, f32 tolerance, f32 p, f32 epsilon)
	{
		assert(!target.Empty());
		assert(!source.Empty());
		assert(target.Size() == nn.Size());
		assert(maxIterations >= 1);
		assert(tolerance > 0.0f);
		assert(p > 0.0f && p <= 1.0f);
		assert(epsilon > 0.0f);

		const index_t N = source.Size();

		ICPResult result;
		result.transform = { glm::mat3(1.0f), glm::vec3(0.0f) };

		std::vector<index_t> correspondences(N, 0);
		std::vector<f32> weights(N, 1.0f);

		f32 prevError = F32_MAX;

		TimePoint startTime = Clock::now();

		for (u32 iter = 0; iter < maxIterations; ++iter)
		{
			// Step 1: correspondences
			TimePoint startCorrTime = Clock::now();
			nn.QueryBatch(source.GetStorage(), correspondences);
			TimePoint endCorrTime = Clock::now();
			result.avgCorrespondenceTime_ms += TimeDifference_ms(endCorrTime, startCorrTime);

#ifdef IRLS

			// Step 2: sparse weights from previous residuals
			for (index_t i = 0; i < N; ++i)
			{
				glm::vec3 r = source[i] - target[correspondences[i]];
				f32 ri = glm::length(r);

				// IRLS weight for ||r||^p
				weights[i] = 1.0f / std::pow(ri + epsilon, 2.0f - p);
				//weights[i] = std::clamp(weights[i], 1e-3f, 1.0f);
			}

			// Optional normalization for stability
			f32 maxW = 0.0f;
			for (f32 w : weights) maxW = std::max(maxW, w);
			if (maxW > 0.0f)
			{
				for (f32& w : weights) w /= maxW;
			}

			// Step 3: weighted rigid solve
			TimePoint startSolveTime = Clock::now();
			RigidTransform transform = WeightedPointToPoint(target, source, correspondences, weights);
			TimePoint endSolveTime = Clock::now();
			result.avgSolverTime_ms += TimeDifference_ms(endSolveTime, startSolveTime);

#endif // IRLS

			// Step 4: apply transform
			source.Transform(transform);

			result.transform.translation = transform.rotation * result.transform.translation + transform.translation;
			result.transform.rotation = transform.rotation * result.transform.rotation;

			// Step 5: compute sparse-style RMS
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
