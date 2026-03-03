#include <limits>
#include <chrono>
#include <assert.h>
#include <Eigen/Dense>
#include "GeometricRegistration.h"

namespace geo
{
	namespace detail
	{
		static inline glm::mat3 EigenToGlm(const Eigen::Matrix3f& m)
		{
			glm::mat3 result(0.0f);
			for (int r = 0; r < 3; ++r)
				for (int c = 0; c < 3; ++c)
					result[c][r] = m(r, c); // column-major

			return result;
		}

		static inline Eigen::Matrix3f GlmToEigen(const glm::mat3& m)
		{
			Eigen::Matrix3f result;
			for (int r = 0; r < 3; ++r)
				for (int c = 0; c < 3; ++c)
					result(r, c) = m[c][r];

			return result;
		}
	}
	
	using Clock = std::chrono::steady_clock;
	using TimePoint = Clock::time_point;

	// Compute a - b in milliseconds
	static double TimeDifference_ms(TimePoint a, TimePoint b)
	{
		return std::chrono::duration<double, std::milli>(a - b).count();
	}

	ICPResult NaiveICP(const INearestNeighbor& targetSurface, PointCloud3D& source, int maxIterations, float tolerance)
	{
		assert(source.Count() > 1);
		assert(maxIterations >= 1);
		assert(tolerance > 0.0f);

		ICPResult result;
		result.transform = { glm::mat3(1.0f), glm::vec3(0.0f) };

		float prevError = std::numeric_limits<float>::max();

		std::vector<glm::vec3> correspondences;
		correspondences.reserve(source.Count());

		TimePoint startTime = Clock::now();

		for (int iter = 0; iter < maxIterations; iter++)
		{
			correspondences.clear();

			// Find correspondences
			TimePoint startCorrTime = Clock::now();
			for (const auto& p : source)
			{
				correspondences.emplace_back(targetSurface.FindClosestPoint(p));
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
			for (size_t i = 0; i < source.Count(); ++i)
			{
				glm::vec3 p = source[i] - centroidSrc;
				glm::vec3 q = correspondences[i] - centroidCor;
				H += glm::outerProduct(p, q);
			}

			// SVD
			Eigen::Matrix3f H_e = detail::GlmToEigen(H);

			Eigen::JacobiSVD<Eigen::Matrix3f> svd(
				H_e,
				Eigen::ComputeFullU | Eigen::ComputeFullV);

			Eigen::Matrix3f R_e =
				svd.matrixV() * svd.matrixU().transpose();

			// Reflection fix
			if (R_e.determinant() < 0.0f)
			{
				Eigen::Matrix3f V = svd.matrixV();
				V.col(2) *= -1.0f;
				R_e = V * svd.matrixU().transpose();
			}

			glm::mat3 R = detail::EigenToGlm(R_e);

			Eigen::Vector3f cSrc(centroidSrc.x, centroidSrc.y, centroidSrc.z);
			Eigen::Vector3f cDst(centroidCor.x, centroidCor.y, centroidCor.z);

			Eigen::Vector3f t_e = cDst - R_e * cSrc;
			glm::vec3 t(t_e.x(), t_e.y(), t_e.z());

			TimePoint endSolveTime = Clock::now();
			result.avgSolverTime_ms += TimeDifference_ms(endSolveTime, startSolveTime);

			// Apply transform
			source.Transform(R, t);

			result.transform.translation = R * result.transform.translation + t;
			result.transform.rotation = R * result.transform.rotation;

			// Compute RMS
			float error = 0.0f;
			for (size_t i = 0; i < source.Count(); ++i)
			{
				glm::vec3 diff = source[i] - correspondences[i];
				error += glm::dot(diff, diff);
			}

			error = std::sqrt(error / source.Count());
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

