#include <iostream>
#include <limits>
#include <assert.h>
#include <array>
#include <Eigen/Dense>
#include "GeometricRegistration.h"

namespace geo
{
	// point-to-point ICP using Euclidial distace
	float NaiveICP(const PointCloud3D& target, PointCloud3D& source, int max_interrations)
	{

		std::vector<glm::vec3> correspondences_points;
		correspondences_points.reserve(source.Count());

		glm::mat3 final_rot(1.0f);
		glm::vec3 final_t(0.0f);
		float prevError = std::numeric_limits<float>::max();

		for (int i = 0; i < max_interrations; i++)
		{
			// Find Closest Points
			correspondences_points.clear();
			for (auto& p : source)
			{
                correspondences_points.emplace_back(target.FindClosestPoint(p));
			}
			// make the correspondences into a point cloud
            geo::PointCloud3D correspondences(std::move(correspondences_points));

			// Compute Best Rigid Transform
			glm::vec3 centroidSource = source.Centroid();
			glm::vec3 centroidTarget = correspondences.Centroid();

			glm::mat3 H(0.0f);
			for (int j = 0; j < source.Count(); j++)
			{
				glm::vec3 p = source[j] - centroidSource;
				glm::vec3 q = correspondences[j] - centroidTarget;

				H += glm::outerProduct(p, q);
			}

			Eigen::Matrix3f H_eigen;
			for (int r = 0; r < 3; r++)
				for (int c = 0; c < 3; c++)
					H_eigen(r, c) = H[c][r]; // GLM is column-major

			Eigen::JacobiSVD<Eigen::Matrix3f> svd(
				H_eigen,
				Eigen::ComputeFullU | Eigen::ComputeFullV
			);

			Eigen::Matrix3f R_eigen =
				svd.matrixV() * svd.matrixU().transpose();

			// Handle reflection case
			if (R_eigen.determinant() < 0.0f)
			{
				Eigen::Matrix3f V = svd.matrixV();
				V.col(2) *= -1.0f;
				R_eigen = V * svd.matrixU().transpose();
			}

			// Compute translation
			Eigen::Vector3f centroidSource_e(centroidSource.x, centroidSource.y, centroidSource.z);
			Eigen::Vector3f centroidTarget_e(centroidTarget.x, centroidTarget.y, centroidTarget.z);

			Eigen::Vector3f t_e = centroidTarget_e - R_eigen * centroidSource_e;

			glm::mat3 R_glm(0.0f);
			for (int r = 0; r < 3; r++)
				for (int c = 0; c < 3; c++)
					R_glm[c][r] = R_eigen(r, c); // back to GLM column-major

			glm::vec3 t_glm(t_e.x(), t_e.y(), t_e.z());

			source.Transform(R_glm, t_glm);
			final_rot *= R_glm;
			final_t += t_glm;

			// Compute Error
			float error = 0.0f;
			for (int j = 0; j < source.Count(); j++)
			{
				float dist = glm::distance(source[j], correspondences[j]);
				error += dist * dist;
			}

			error = std::sqrt(error / source.Count());

			std::cout << "Iteration " << i << " | RMS Error: " << error << std::endl;

			if (std::abs(prevError - error) < 1e-5f)
			{
				std::cout << "Converged.\n";
				break;
			}

			prevError = error;
		}

		//return { final_rot, final_t };
		return prevError;
	}

}

