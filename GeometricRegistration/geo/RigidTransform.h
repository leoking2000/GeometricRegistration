#pragma once
#include <glm/glm.hpp>

namespace geo
{
	struct RigidTransform
	{
		glm::mat3 rotation;
		glm::vec3 translation;
	};
}
