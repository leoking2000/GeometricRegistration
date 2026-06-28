#include "ViewerCamera.h"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>

namespace gl
{
    void ViewerCamera::Orbit(geo::f32 deltaYaw, geo::f32 deltaPitch)
    {
        m_yaw += deltaYaw;
        const geo::f32 limit = glm::radians(89.0f);
        m_pitch = glm::clamp(m_pitch + deltaPitch, -limit, limit);
    }

    void ViewerCamera::Zoom(geo::f32 deltaDistance)
    {
        m_distance += deltaDistance;
        m_distance = glm::max(0.01f, m_distance);
    }

    void ViewerCamera::Pan(const glm::vec3& delta)
    {
        m_target += delta;
    }

    void ViewerCamera::Pan(geo::f32 dx, geo::f32 dy)
    {
        m_target += Right() * dx;
        m_target += Up() * dy;
    }

    glm::vec3 ViewerCamera::Position() const
    {
        const geo::f32 cp = glm::cos(m_pitch); const geo::f32 cy = glm::cos(m_yaw);
        const geo::f32 sp = glm::sin(m_pitch); const geo::f32 sy = glm::sin(m_yaw);

        glm::vec3 offset = {};

        offset.x = m_distance * cp * sy;
        offset.y = m_distance * sp;
        offset.z = m_distance * cp * cy;

        return m_target + offset;
    }

    glm::mat4 ViewerCamera::ViewMatrix() const
    {
        return glm::lookAt(Position(), m_target, glm::vec3(0.0f, 1.0f, 0.0f));
    }

    glm::mat4 ViewerCamera::ProjectionMatrix(geo::f32 aspectRatio) const
    {
        return glm::perspective(m_fov, aspectRatio, m_nearPlane, m_farPlane);
    }

    glm::vec3 ViewerCamera::Forward() const
    {
        return glm::normalize(m_target - Position());
    }

    glm::vec3 ViewerCamera::Right() const
    {
        return glm::normalize(glm::cross(Forward(), glm::vec3(0.0f, 1.0f, 0.0f)));
    }

    glm::vec3 ViewerCamera::Up() const
    {
        return glm::normalize(glm::cross(Right(), Forward()));
    }
}
