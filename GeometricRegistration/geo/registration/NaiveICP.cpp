#include "math/SVD.h"
#include "NaiveICP.h"

namespace geo
{
	using Clock = std::chrono::steady_clock;
	using TimePoint = Clock::time_point;

	// Compute a - b in milliseconds
	static f64 TimeDifference_ms(TimePoint end, TimePoint start)
	{
		return std::chrono::duration<f64, std::milli>(end - start).count();
	}

	ICPResult NaiveICP(const PointCloud3D& target, PointCloud3D& source, const INearestNeighbor& nn, 
		u32 maxIterations, f32 tolerance, bool useNormals)
	{
		assert(!source.Empty());
		assert(!target.Empty());
		assert(target.Size() == nn.Size());
		assert(maxIterations >= 1);
		assert(tolerance > 0.0f);

		index_t numberOfPoints = source.Size();

		ICPResult result;
		result.transform = { glm::mat3(1.0f), glm::vec3(0.0f) };

		f32 prevError = F32_MAX;

		std::vector<index_t> correspondences;
		correspondences.resize(numberOfPoints, 0);

		TimePoint startTime = Clock::now();


		for (u32 iter = 0; iter < maxIterations; iter++)
		{
			// Find correspondences 
			TimePoint startCorrTime = Clock::now();

			nn.QueryBatch(source.GetStorage(), correspondences);

			TimePoint endCorrTime = Clock::now();
			result.avgCorrespondenceTime_ms += TimeDifference_ms(endCorrTime, startCorrTime);

			// Compute centroids
			glm::vec3 centroidSrc = source.Centroid();
			glm::vec3 centroidCor(0.0f);
			for (index_t i : correspondences)
			{
				centroidCor += target[i];
			}
			centroidCor = centroidCor / (float)numberOfPoints;

			TimePoint startSolveTime = Clock::now();

			// Compute covariance matrix
			glm::mat3 H(0.0f);
			for (index_t i = 0; i < numberOfPoints; ++i)
			{
				glm::vec3 p = source[i] - centroidSrc;
				glm::vec3 q = target[correspondences[i]] - centroidCor;
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
			f32 error = 0.0f;
			for (index_t i = 0; i < numberOfPoints; ++i)
			{
				glm::vec3 diff = source[i] - target[correspondences[i]];
				error += glm::dot(diff, diff);
			}

			error = std::sqrt(error / numberOfPoints);
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
