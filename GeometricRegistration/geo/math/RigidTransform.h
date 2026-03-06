#pragma once
#include <glm/glm.hpp>

namespace geo
{
	struct RigidTransform
	{
		glm::mat3 rotation    = glm::mat3(1.0f);
		glm::vec3 translation = glm::vec3(0.0f);
	};
}
