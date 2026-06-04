#include "MeshDrawable.h"

namespace gl
{
	void MeshDrawable::Upload(const geo::Mesh& mesh)
	{
		std::vector<float> vertex_buffer;

		for (geo::index_t i = 0; i < mesh.VertexCount(); i++)
		{
			glm::vec3 pos    = mesh.Vertex(i);
			glm::vec3 normal = mesh.Normal(i);

			vertex_buffer.emplace_back(pos.x);
			vertex_buffer.emplace_back(pos.y);
			vertex_buffer.emplace_back(pos.z);

			vertex_buffer.emplace_back(normal.x);
			vertex_buffer.emplace_back(normal.y);
			vertex_buffer.emplace_back(normal.z);
		}

		VertexBuffer vertexBuffer((const void*)vertex_buffer.data(), (geo::u32)(vertex_buffer.size() * sizeof(float)));

		ElementType arr[2] = { ElementType::FLOAT3, ElementType::FLOAT3_N };
		Layout<2> layout(arr);

		VertexArray vertexArray;
		vertexArray.AddBuffer(std::move(vertexBuffer), layout);

		std::vector<geo::u32> indices;

		for (geo::index_t i = 0; i < mesh.TriangleCount(); i++)
		{
			const geo::TriangleData& tri = mesh.Triangle(i);

			indices.emplace_back(tri[0]);
			indices.emplace_back(tri[1]);
			indices.emplace_back(tri[2]);
		}

		IndexBuffer indexBuffer(indices.data(), (geo::u32)indices.size());

		m_gpuMesh = gl::Mesh(vertexArray, indexBuffer, 2);
	}

	void MeshDrawable::Draw(
		const gl::ShaderProgram& shader, 
		const glm::mat4& mvp, const glm::mat4& model, 
		const glm::vec3 color, const glm::vec3& cameraPos, 
		const glm::vec3& lightDir)
	{
		shader.Bind();

		shader.SetUniform("u_mvp", mvp);
		shader.SetUniform("u_model", model);

		shader.SetUniform("u_color", color);

		shader.SetUniform("u_cameraPos", cameraPos);

		shader.SetUniform("u_lightDir", glm::normalize(lightDir));

		m_gpuMesh.Draw();

		shader.UnBind();
	}
}
