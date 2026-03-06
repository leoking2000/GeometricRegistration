#include "math/SVD.h"
#include "NaiveICP.h"

namespace geo
{
	using Clock = std::chrono::steady_clock;
	using TimePoint = Clock::time_point;

	// Compute a - b in milliseconds
	static double TimeDifference_ms(TimePoint end, TimePoint start)
	{
		return std::chrono::duration<double, std::milli>(end - start).count();
	}


	ICPResult NaiveICP(const INearestNeighbor& target, PointCloud3D& source, int maxIterations, float tolerance)
	{
		assert(source.Size() > 1);
		assert(maxIterations >= 1);
		assert(tolerance > 0.0f);

		ICPResult result;
		result.transform = { glm::mat3(1.0f), glm::vec3(0.0f) };

		float prevError = std::numeric_limits<float>::max();

		std::vector<glm::vec3> correspondences;
		correspondences.reserve(source.Size());

		TimePoint startTime = Clock::now();

		for (int iter = 0; iter < maxIterations; iter++)
		{
			correspondences.clear();

			// Find correspondences 
			TimePoint startCorrTime = Clock::now();
			for (const auto& p : source)
			{
				correspondences.emplace_back(target.FindClosestPoint(p)); // TODO: batch queries
			}

			TimePoint endCorrTime = Clock::now();
			result.avgCorrespondenceTime_ms += TimeDifference_ms(endCorrTime, startCorrTime);

			// Compute centroids
			glm::vec3 centroidSrc = source.Centroid();
			glm::vec3 centroidCor(0.0f);
			for (const auto& p : correspondences)
			{
				centroidCor += p;
			}
			centroidCor = centroidCor / (float)correspondences.size();

			TimePoint startSolveTime = Clock::now();

			// Compute covariance matrix
			glm::mat3 H(0.0f);
			for (size_t i = 0; i < source.Size(); ++i)
			{
				glm::vec3 p = source[i] - centroidSrc;
				glm::vec3 q = correspondences[i] - centroidCor;
				H += glm::outerProduct(p, q);
			}

			// SVD
			SVDResult svd = SVD(H);

			glm::mat3 Ut = glm::transpose(svd.U);
			glm::mat3 R = svd.V * Ut;

			// Reflection fix
			if (glm::determinant(R) < 0.0f)
			{
				svd.V[2] *= -1.0f;
				R = svd.V * Ut;
			}

			glm::vec3 t = centroidCor - R * centroidSrc;

			TimePoint endSolveTime = Clock::now();
			result.avgSolverTime_ms += TimeDifference_ms(endSolveTime, startSolveTime);

			// Apply transform
			source.Transform(R, t);

			result.transform.translation = R * result.transform.translation + t;
			result.transform.rotation = R * result.transform.rotation;

			// Compute RMS
			float error = 0.0f;
			for (size_t i = 0; i < source.Size(); ++i)
			{
				glm::vec3 diff = source[i] - correspondences[i];
				error += glm::dot(diff, diff);
			}

			error = std::sqrt(error / source.Size());
			result.rms = error;
			result.iterations = iter + 1;

			if (std::abs(prevError - error) < tolerance)
			{
				result.converged = true;
				break;
			}

			prevError = error;
		}

		TimePoint endTime = Clock::now();
		result.totalElapsed_ms = TimeDifference_ms(endTime, startTime);
		result.avgCorrespondenceTime_ms /= result.iterations;
		result.avgSolverTime_ms /= result.iterations;

		return result;
	}

}
