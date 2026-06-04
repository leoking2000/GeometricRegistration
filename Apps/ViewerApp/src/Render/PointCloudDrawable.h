#pragma once
#include <geo/geometry/PointCloud3D.h>
#include <Graphics/Shader.h>

namespace gl
{
    class PointCloudDrawable
    {
    public:
        void Upload(const geo::PointCloud3D& cloud);
        void Draw(const ShaderProgram& shader,
            const glm::mat4& mvp,
            const glm::vec3 color,
            geo::f32 pointSize = 3.0f) const;
    private:
        geo::u32 m_vao   = 0;
        geo::u32 m_vbo   = 0;
        geo::u32 m_count = 0;
    };
}
