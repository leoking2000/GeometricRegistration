#include <Eigen/Dense>
#include "SVD.h"

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

	SVDResult SVD(const glm::mat3& A)
	{
		Eigen::Matrix3f A_eigen = detail::GlmToEigen(A);

		Eigen::JacobiSVD<Eigen::Matrix3f> svd(A_eigen,Eigen::ComputeFullU | Eigen::ComputeFullV);

		SVDResult result{};
		result.U = detail::EigenToGlm(svd.matrixU());
		result.V = detail::EigenToGlm(svd.matrixV());

		Eigen::Vector3f s = svd.singularValues();
		result.S = glm::vec3(s(0), s(1), s(2));

		return result;
	}
}
