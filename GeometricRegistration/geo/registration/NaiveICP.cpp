#include "math/SVD.h"
#include "math/LinearSolve.h"
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

	static inline RigidTransform PointToPoint(const PointCloud3D& target, const PointCloud3D& source, const std::vector<index_t>& correspondences)
	{
		index_t numberOfPoints = source.Size();

		// Compute centroids
		glm::vec3 centroidSrc = source.Centroid();
		glm::vec3 centroidCor(0.0f);
		for (index_t i : correspondences)
		{
			centroidCor += target[i];
		}
		centroidCor = centroidCor / (float)numberOfPoints;

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

		return { R, t };
	}

	static inline glm::mat3 Skew(const glm::vec3& v)
	{
		return glm::mat3{
			{ 0.0f, -v.z, v.y },
			{ v.z, 0.0f, -v.x },
			{ -v.y, v.x, 0.0f }
		};
	}

	static inline glm::mat3 Rodrigues(const glm::vec3& omega)
	{
		float theta = glm::length(omega);
		if(theta < 1e-8f) return glm::mat3(1.0f) + Skew(omega);
		glm::vec3 k = omega / theta;
		glm::mat3 K = Skew(k); // Skew is cross-product matrix
		return glm::mat3(1.0f) + std::sin(theta) * K + (1 - std::cos(theta)) * K * K;
	}

	static inline RigidTransform PointToPlane(const PointCloud3D& target, const PointCloud3D& source, const std::vector<index_t>& correspondences)
	{
		index_t numberOfPoints = source.Size();

		// Build 6x6 system
		geo::Mat6 AtA{};
		geo::Vec6 Atb{};

		f32 error = 0.0f;

		for (index_t i = 0; i < numberOfPoints; ++i)
		{
			const glm::vec3& p = source.Point(i);
			const glm::vec3& q = target.Point(correspondences[i]);

			// Get normal of target point
			glm::vec3 n = target.Normal(correspondences[i]);

			// Compute b_i = dot(n, q - p)
			f32 bi = glm::dot(n, q - p);

			// Compute rotation part: cross(n, p)
			glm::vec3 crossPn = glm::cross(n, p);

			// Fill AtA and Atb
			std::array<f32, 6> row{ crossPn.x, crossPn.y, crossPn.z, n.x, n.y, n.z };

			// Accumulate AtA = sum row^T * row
			for (int r = 0; r < 6; ++r)
				for (int c = 0; c < 6; ++c)
					AtA[r][c] += row[r] * row[c];

			// Accumulate Atb = sum row * b_i
			for (int r = 0; r < 6; ++r)
				Atb[r] += row[r] * bi;

			error += bi * bi; // point-to-plane error
		}

		//for (int i = 0; i < 6; i++)
			//AtA[i][i] += 1e-6f;

		// Solve 6x6 system
		geo::Vec6 x = geo::Solve6x6(AtA, Atb);

		glm::vec3 rotVec(x[0], x[1], x[2]);
		glm::vec3 t(x[3], x[4], x[5]);

		// Convert small rotation vector to rotation matrix
		glm::mat3 R = Rodrigues(rotVec);

		return { R, t };
	}

	ICPResult NaiveICP(const PointCloud3D& target, PointCloud3D& source, const INearestNeighbor& nn, u32 maxIterations, f32 tolerance, bool useNormals)
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
			
			// Solve System
			TimePoint startSolveTime = Clock::now();

			RigidTransform transform;
			if(useNormals && target.HasNormals())
			{
				transform = PointToPlane(target, source, correspondences);
			}
			else
			{
				transform = PointToPoint(target, source, correspondences);
			}

			TimePoint endSolveTime = Clock::now();
			result.avgSolverTime_ms += TimeDifference_ms(endSolveTime, startSolveTime);

			// Apply transform
			source.Transform(transform);

			result.transform.translation = transform.rotation * result.transform.translation + transform.translation;
			result.transform.rotation = transform.rotation * result.transform.rotation;

			// Compute RMS
			f32 error = 0.0f;
			for (index_t i = 0; i < numberOfPoints; ++i)
			{
				if(useNormals && target.HasNormals())
					error += std::pow(glm::dot(source[i]-target[correspondences[i]], target.Normal(correspondences[i])), 2);
				else
					error += glm::dot(source[i]-target[correspondences[i]], source[i]-target[correspondences[i]]);
			}

			error = std::sqrt(error / numberOfPoints);
			result.rms = error;
			result.iterations = iter + 1;

			// converged check
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
