#include <Graphics/LeoGraphics.h>
#include <geo/io/IOGeometry.h>
#include <geo/logging/LogMacros.h>
#include "PointCloudDrawable.h"

static_assert(sizeof(glm::vec3) == 3 * sizeof(float));

namespace gl
{
    PointCloudDrawable::PointCloudDrawable(const std::vector<glm::vec3>& points)
    {
        Upload(points);
    }

    void PointCloudDrawable::Upload(const std::vector<glm::vec3>& points)
    {
        m_pointCount = 0; // reset pointCount

        // Create a new VertexArray
        m_vao = VertexArray();

        // hudle empty case
        if (points.empty())
        {
            m_transform = geo::RigidTransform::Identity();
            GEOLOGWARN("Can not upload cloud with zero data.");
            return;
        }

        // Create VertexBuffer
        m_pointCount = (geo::u32)points.size();
        gl::VertexBuffer vbo(
            (const void*)points.data(), 
            (geo::u32)(points.size() * sizeof(glm::vec3)),
            gl::BufferUsage::Static
        );

        // Create VertexBuffer Layout
        gl::ElementType arr[1] = { gl::ElementType::FLOAT3 };
        gl::Layout<1> layout(arr);

        // add VertexBuffer to VertexArray
        m_vao.AddBuffer(std::move(vbo), layout);

        m_transform = geo::RigidTransform::Identity(); // reset transform

        GEOLOGINFO("Uploaded PointCloudDrawable | " << m_pointCount << " points");
    }

    void PointCloudDrawable::Draw(const gl::ShaderProgram& shader,
        const glm::mat4& view_projection, const glm::vec3& color, geo::f32 pointSize) const
	{
        if (m_pointCount == 0) {
            return;
        }

        shader.Bind();

        shader.SetUniform("u_mvp", view_projection * m_transform.ToMat4());
        shader.SetUniform("u_pointSize", pointSize);
        shader.SetUniform("u_color", color);

        m_vao.Bind();

        glDrawArrays(GL_POINTS, 0, (GLsizei)m_pointCount);

        m_vao.UnBind();

        shader.UnBind();
	}

    void PointCloudDrawable::ApplyTransform(const geo::RigidTransform& transform)
    {
        m_transform = geo::RigidTransform::Compose(transform, m_transform);
    }
}
