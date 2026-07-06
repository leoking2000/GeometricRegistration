#include "MeshDrawable.h"

namespace gl
{
	MeshDrawable::MeshDrawable(
		const std::vector<glm::vec3>&  points, 
		const std::vector<glm::vec3>&  normals, 
		const std::vector<glm::uvec3>& triangles)
	{
		Upload(points, normals, triangles);
	}

	void MeshDrawable::Upload(
		const std::vector<glm::vec3>&  points, 
		const std::vector<glm::vec3>&  normals, 
		const std::vector<glm::uvec3>& triangles)
	{
		assert(points.size() == normals.size());

		std::vector<float> vertex_buffer;
		vertex_buffer.reserve(points.size() * 6);

		for (geo::index_t i = 0; i < points.size(); i++)
		{
			glm::vec3 pos    = points[i];
			glm::vec3 normal = normals[i];

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
		indices.reserve(triangles.size() * 3);

		for (geo::index_t i = 0; i < triangles.size(); i++)
		{
			const glm::uvec3& tri = triangles[i];

			indices.emplace_back(tri[0]);
			indices.emplace_back(tri[1]);
			indices.emplace_back(tri[2]);
		}

		IndexBuffer indexBuffer(indices.data(), (geo::u32)indices.size());
		m_gpuMesh = gl::Mesh(vertexArray, indexBuffer, 2);
	}

	void MeshDrawable::Draw(
		const gl::ShaderProgram& shader, 
		const glm::mat4& view_projection,
		const glm::vec3 color, const glm::vec3& cameraPos, 
		const glm::vec3& lightDir)
	{
		shader.Bind();

		glm::mat4 model = m_transform.ToMat4();

		shader.SetUniform("u_mvp", view_projection * model);
		shader.SetUniform("u_model", model);

		shader.SetUniform("u_color", color);

		shader.SetUniform("u_cameraPos", cameraPos);

		shader.SetUniform("u_lightDir", glm::normalize(lightDir));

		m_gpuMesh.Draw();

		shader.UnBind();
	}

	void MeshDrawable::ApplyTransform(const geo::RigidTransform& transform)
	{
		m_transform = geo::RigidTransform::Compose(transform, m_transform);
	}
}
