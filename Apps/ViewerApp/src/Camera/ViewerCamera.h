#pragma once
#include <glm/glm.hpp>
#include <geo/GeoTypes.h>

namespace gl
{
	class ViewerCamera
	{
	public:
		ViewerCamera() = default;
	public:
		void Orbit(geo::f32 deltaYaw, geo::f32 deltaPitch);
		void Zoom(geo::f32 deltaDistance);
		void Pan(const glm::vec3& delta);
		void Pan(geo::f32 dx, geo::f32 dy);
	public:
		glm::vec3 Position() const;
		glm::mat4 ViewMatrix() const;
		glm::mat4 ProjectionMatrix(geo::f32 aspectRatio) const;
	public:
		glm::vec3 Forward() const;
		glm::vec3 Right() const;
		glm::vec3 Up() const;
	public:
		glm::vec3 m_target = glm::vec3(0.0f); // Focus point

		geo::f32 m_distance  = 10.0f;
		geo::f32 m_yaw       = glm::radians(45.0f);
		geo::f32 m_pitch     = glm::radians(25.0f);
		geo::f32 m_fov       = glm::radians(90.0f);
		geo::f32 m_nearPlane = 0.01f;
		geo::f32 m_farPlane  = 500.0f;
	};
}
