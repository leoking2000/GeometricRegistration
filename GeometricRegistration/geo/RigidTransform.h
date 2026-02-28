#pragma once
#include <glm/glm.hpp>

namespace geo
{
	struct RigidTransform
	{
		glm::mat3 rot;
		glm::vec3 t;
	};
}
