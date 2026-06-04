#pragma once
#include <geo/geometry/Mesh.h>
#include <glm/glm.hpp>
#include <Graphics/Mesh.h>
#include <Graphics/Shader.h>
#include <Platform/ViewerCamera.h>

namespace gl
{
	class MeshDrawable
	{
	public:
		MeshDrawable() = default;
	public:
		void Upload(const geo::Mesh& mesh);
		void Draw(const gl::ShaderProgram& shader, 
			const glm::mat4& mvp, const glm::mat4& model, 
			const glm::vec3 color, // TODO: change this to material
			const glm::vec3& cameraPos, const glm::vec3& lightDir = glm::normalize(glm::vec3(-1.0f, -1.0f, -0.5f))
		);
	private:
		Mesh m_gpuMesh;
	};
}
