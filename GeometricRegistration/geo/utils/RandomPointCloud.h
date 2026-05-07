#pragma once
#include <geo/geometry/PointCloud3D.h>
#include "GeoRand.h"

namespace geo
{
	// Generates a synthetic random 3D point cloud distributed on the SURFACE of a rectangular box.
	//
	// The box is defined by its center and dimensions (width, height, depth).
	// Points are uniformly sampled across the six faces of the box (not the interior volume).
	//
	// @param center      Center position of the rectangular box in 3D space
	// @param width       Size of the box along the X-axis
	// @param height      Size of the box along the Y-axis
	// @param depth       Size of the box along the Z-axis
	// @param pointCount  Number of surface points to generate
	// @param rng         Random number generator used for sampling
	// @param haveNormals If true, generates outward-facing normals per point (face-dependent)
	//
	// @return A PointCloud3D containing randomly distributed surface points (and optionally normals)
	PointCloud3D GenerateRandomPointCloudRect(const glm::vec3& center, f32 width, f32 height, f32 depth, u32 pointCount,
		Random& rng, bool haveNormals = true);
}