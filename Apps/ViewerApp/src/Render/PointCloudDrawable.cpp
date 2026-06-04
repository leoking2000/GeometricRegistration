#include "PointCloudDrawable.h"
#include <Graphics/LeoGraphics.h>

namespace gl
{
	void PointCloudDrawable::Upload(const geo::PointCloud3D& cloud)
	{
        std::vector<float> buffer;
        buffer.reserve(cloud.Size() * 3);

        for (geo::index_t i = 0; i < cloud.Size(); i++)
        {
            const glm::vec3& p = cloud.GetPoints()[i];
            buffer.push_back(p.x);
            buffer.push_back(p.y);
            buffer.push_back(p.z);
        }

        m_count = cloud.Size();

        if (m_vao == 0)
        {
            glGenVertexArrays(1, &m_vao);
            glGenBuffers(1, &m_vbo);
        }

        glBindVertexArray(m_vao);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo);

        glBufferData(GL_ARRAY_BUFFER,
            buffer.size() * sizeof(float),
            buffer.data(),
            GL_STATIC_DRAW);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(
            0, 3, GL_FLOAT, GL_FALSE,
            3 * sizeof(float),
            (void*)0
        );

        glBindVertexArray(0);
	}

	void PointCloudDrawable::Draw(const ShaderProgram & shader, 
        const glm::mat4 & mvp, const glm::vec3 color, geo::f32 pointSize) const
	{
        shader.Bind();
        shader.SetUniform("u_mvp", mvp);
        shader.SetUniform("u_pointSize", pointSize);
        shader.SetUniform("u_color", color);

        glBindVertexArray(m_vao);
        glDrawArrays(GL_POINTS, 0, (GLsizei)m_count);
        glBindVertexArray(0);

        shader.UnBind();


	}
}
