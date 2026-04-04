#pragma once
#include <glm/glm.hpp>
#include "utils/GeoTypes.h"

namespace geo
{
	class RigidTransform
	{
	public:
		glm::mat3 rotation    = glm::mat3(1.0f);
		glm::vec3 translation = glm::vec3(0.0f);
	public:
		inline glm::vec3 TransformPoint(const glm::vec3& p) const
		{
			return rotation * p + translation;
		}

		inline glm::vec3 TransformNormal(const glm::vec3& n) const
		{
			return rotation * n;
		}

		inline RigidTransform ComputeInverse() const
		{
			const glm::mat3 Rt = glm::transpose(rotation);

			RigidTransform inv;
			inv.rotation = Rt;
			inv.translation = -(Rt * translation);
			return inv;
		}

		inline glm::mat4 ToMat4() const
		{
			glm::mat4 M(1.0f);

			M[0] = glm::vec4(rotation[0], 0.0f);
			M[1] = glm::vec4(rotation[1], 0.0f);
			M[2] = glm::vec4(rotation[2], 0.0f);
			M[3] = glm::vec4(translation, 1.0f);

			return M;
		}
	public:
		static inline RigidTransform Identity()
		{
			return { glm::mat3(1.0f), glm::vec3(0.0f) };
		}

		// Apply B then A
		static inline RigidTransform Compose(const RigidTransform& A, const RigidTransform& B)
		{
			RigidTransform C;
			C.rotation = A.rotation * B.rotation;
			C.translation = A.rotation * B.translation + A.translation;
			return C;
		}
	};

	inline bool NearlyEqual(
		const RigidTransform& A,
		const RigidTransform& B,
		f32 eps = 1e-5f)
	{
		for (int c = 0; c < 3; ++c)
		{
			for (int r = 0; r < 3; ++r)
			{
				if (std::abs(A.rotation[c][r] - B.rotation[c][r]) > eps)
				{
					return false;
				}
			}
		}

		for (int i = 0; i < 3; ++i)
		{
			if (std::abs(A.translation[i] - B.translation[i]) > eps)
			{
				return false;
			}
		}

		return true;
	}
}
