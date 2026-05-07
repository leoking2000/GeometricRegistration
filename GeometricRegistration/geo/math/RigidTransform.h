#pragma once
#include <cmath>
#include <glm/glm.hpp>
#include "utils/GeoTypes.h"

namespace geo
{
	// Represents a 3D rigid body transform (SE(3)):
	// - rotation: 3x3 orthonormal matrix
	// - translation: 3D offset
	//
	// Assumption: rotation remains orthonormal (no scaling/shearing).
	class RigidTransform
	{
	public:
		glm::mat3 rotation    = glm::mat3(1.0f);
		glm::vec3 translation = glm::vec3(0.0f);
	public:
		RigidTransform() = default;

		inline RigidTransform(glm::mat3 in_rotation, glm::vec3 in_translation)
			: rotation(in_rotation), translation(in_translation)
		{
		}

		// Constructor from raw float array:
		// Assumes layout:
		// x[0..2] -> translation (tx, ty, tz)
		// x[3..5] -> Euler angles (rx, ry, rz) in radians
		//
		// Rotation order: ZYX (yaw-pitch-roll) assumed
		// i.e. R = Rz * Ry * Rx
		inline explicit RigidTransform(const float* x)
		{
			// --- Translation ---
			translation = glm::vec3(x[0], x[1], x[2]);

			// --- Euler angles ---
			const float rx = x[3];
			const float ry = x[4];
			const float rz = x[5];

			// Precompute trig
			const float cx = std::cos(rx), sx = std::sin(rx);
			const float cy = std::cos(ry), sy = std::sin(ry);
			const float cz = std::cos(rz), sz = std::sin(rz);

			// Rotation matrices (Z * Y * X)
			glm::mat3 Rx = {
				1, 0, 0,
				0, cx, -sx,
				0, sx, cx
			};

			glm::mat3 Ry = {
				cy, 0, sy,
				0,  1, 0,
				-sy, 0, cy
			};

			glm::mat3 Rz = {
				cz, -sz, 0,
				sz,  cz, 0,
				0,   0,  1
			};

			rotation = Rz * Ry * Rx;
		}
	public:
		// Applies full rigid transform to a point (rotation + translation)
		inline glm::vec3 TransformPoint(const glm::vec3& p) const
		{
			return rotation * p + translation;
		}


		// Applies only rotation to a normal vector (no translation)
		// Assumes rotation is orthonormal, so no inverse-transpose is needed
		inline glm::vec3 TransformNormal(const glm::vec3& n) const
		{
			return rotation * n;
		}

		// Computes the inverse rigid transform
		inline RigidTransform ComputeInverse() const
		{
			const glm::mat3 Rt = glm::transpose(rotation);

			RigidTransform inv;
			inv.rotation = Rt;
			inv.translation = -(Rt * translation);
			return inv;
		}

		// Converts rigid transform into a 4x4 homogeneous matrix
		// Column-major layout (GLM convention)
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
		// Returns identity transform (no rotation, no translation)
		static inline RigidTransform Identity()
		{
			return { glm::mat3(1.0f), glm::vec3(0.0f) };
		}


		// Composes two rigid transforms:
		// C = A ∘ B meaning: apply B first, then A
		// C(p) = A(B(p))
		static inline RigidTransform Compose(const RigidTransform& A, const RigidTransform& B)
		{
			RigidTransform C;
			C.rotation = A.rotation * B.rotation;
			C.translation = A.rotation * B.translation + A.translation;
			return C;
		}
	};

	// Extracts rotation angle (in radians) from a rotation matrix.
	// Based on trace(R) = 1 + 2cos(theta)
	inline f32 RotationAngle(const glm::mat3& R)
	{
		f32 trace = R[0][0] + R[1][1] + R[2][2];
		f32 cos_theta = (trace - 1.0f) * 0.5f;

		// Clamp for numerical safety
		cos_theta = glm::clamp(cos_theta, -1.0f, 1.0f);

		return std::acos(cos_theta);
	}
}
