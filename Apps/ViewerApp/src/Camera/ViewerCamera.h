#pragma once
#include <glm/glm.hpp>
#include <core/Types.h>

namespace gl
{
	class ViewerCamera
	{
	public:
		ViewerCamera() = default;
	public:
		void Orbit(f32 deltaYaw, f32 deltaPitch);
		void Zoom(f32 deltaDistance);
		void Pan(const glm::vec3& delta);
		void Pan(f32 dx, f32 dy);
	public:
		glm::vec3 Position() const;
		glm::mat4 ViewMatrix() const;
		glm::mat4 ProjectionMatrix(f32 aspectRatio) const;
	public:
		glm::vec3 Forward() const;
		glm::vec3 Right() const;
		glm::vec3 Up() const;
	public:
		glm::vec3 m_target = glm::vec3(0.0f); // Focus point

		f32 m_distance  = 10.0f;
		f32 m_yaw       = glm::radians(45.0f);
		f32 m_pitch     = glm::radians(25.0f);
		f32 m_fov       = glm::radians(90.0f);
		f32 m_nearPlane = 0.01f;
		f32 m_farPlane  = 500.0f;
	};
}
