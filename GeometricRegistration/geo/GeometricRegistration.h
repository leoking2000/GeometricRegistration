#pragma once
#include "PointCloud3D.h"

namespace geo
{
	struct RigidTransform
	{
		glm::mat3 rot;
		glm::vec3 t;
	};

	// point-to-point ICP using Euclidial distace
	float NaiveICP(const PointCloud3D& target, PointCloud3D& source, int max_interrations);
}
