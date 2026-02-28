#pragma once
#include "PointCloud3D.h"

namespace geo
{
	// point-to-point ICP using Euclidial distace
	float NaiveICP(const PointCloud3D& target, PointCloud3D& source, int max_interrations);
}
