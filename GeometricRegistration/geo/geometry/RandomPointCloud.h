#pragma once
#include "utils/GeoRand.h"
#include "PointCloud3D.h"


namespace geo
{
	static PointCloud3D GenerateRandomPointCloudRect(const glm::vec3& center, f32 width, f32 height, f32 depth, u32 pointCount,
		Random& rng, bool haveNormals = true)
	{
		std::vector<glm::vec3> points;
		std::vector<glm::vec3> normals;

		points.reserve(pointCount);
		normals.reserve(pointCount);

		f32 hx = width * 0.5f;
		f32 hy = height * 0.5f;
		f32 hz = depth * 0.5f;

		for (u32 i = 0; i < pointCount; ++i)
		{
			u32 face = rng.UInt(0, 5);

			glm::vec3 p;
			glm::vec3 n;

			switch (face)
			{
			case 0: // +X
				p.x = center.x + hx;
				p.y = center.y + rng.Float(-hy, hy);
				p.z = center.z + rng.Float(-hz, hz);
				n = glm::vec3(1, 0, 0);
				break;

			case 1: // -X
				p.x = center.x - hx;
				p.y = center.y + rng.Float(-hy, hy);
				p.z = center.z + rng.Float(-hz, hz);
				n = glm::vec3(-1, 0, 0);
				break;

			case 2: // +Y
				p.x = center.x + rng.Float(-hx, hx);
				p.y = center.y + hy;
				p.z = center.z + rng.Float(-hz, hz);
				n = glm::vec3(0, 1, 0);
				break;

			case 3: // -Y
				p.x = center.x + rng.Float(-hx, hx);
				p.y = center.y - hy;
				p.z = center.z + rng.Float(-hz, hz);
				n = glm::vec3(0, -1, 0);
				break;

			case 4: // +Z
				p.x = center.x + rng.Float(-hx, hx);
				p.y = center.y + rng.Float(-hy, hy);
				p.z = center.z + hz;
				n = glm::vec3(0, 0, 1);
				break;

			case 5: // -Z
				p.x = center.x + rng.Float(-hx, hx);
				p.y = center.y + rng.Float(-hy, hy);
				p.z = center.z - hz;
				n = glm::vec3(0, 0, -1);
				break;
			}

			points.push_back(p);
			normals.push_back(n);
		}

		PointCloud3D cloud(std::move(points), haveNormals ? std::move(normals) : std::vector<glm::vec3>());

		return cloud;
	}
}