#pragma once
#include <glm/glm.hpp>
#include <core/math/RigidTransform.h>
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
		void ApplyTransform(const core::RigidTransform& transform);
		inline void SetTransform(const core::RigidTransform& transform) { m_transform = transform; }
		inline const core::RigidTransform& GetTransform() const { return m_transform; }
	private:
		core::RigidTransform m_transform = core::RigidTransform::Identity();
		gl::Mesh m_gpuMesh;
	};
}
