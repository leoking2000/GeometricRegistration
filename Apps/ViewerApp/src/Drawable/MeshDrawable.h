#pragma once
#include <glm/glm.hpp>
#include <geo/math/RigidTransform.h>
#include <Graphics/Mesh.h>
#include <Graphics/Shader.h>
#include <Graphics/Texture.h>

namespace gl
{
	class MeshDrawable final
	{
	public:
		MeshDrawable() = default;

		MeshDrawable(
			const std::vector<glm::vec3>&  points, 
			const std::vector<glm::vec3>&  normals,
			const std::vector<glm::uvec3>& triangles
		);
	public:
		void Upload(
			const std::vector<glm::vec3>& points,
			const std::vector<glm::vec3>& normals,
			const std::vector<glm::uvec3>& triangles
		);
	public:
		void Draw(const gl::ShaderProgram& shader, 
			const glm::mat4& view_projection,
			const glm::vec3 color, // TODO: change this to material
			const glm::vec3& cameraPos, const glm::vec3& lightDir = glm::normalize(glm::vec3(-1.0f, -1.0f, -0.5f))
		);
	public:
		// Applies an additional rigid transform to the drawable.
		// The supplied transform is composed with the existing model transform.
		void ApplyTransform(const geo::RigidTransform& transform);
		inline void SetTransform(const geo::RigidTransform& transform) { m_transform = transform; }
		inline const geo::RigidTransform& GetTransform() const { return m_transform; }
	private:
		geo::RigidTransform m_transform = geo::RigidTransform::Identity();
		gl::Mesh m_gpuMesh;
	};
}
