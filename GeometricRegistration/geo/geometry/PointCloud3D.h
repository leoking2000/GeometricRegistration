#pragma once
#include <vector>
#include "geo/math/RigidTransform.h"
#include "core/GeoTypes.h"
#include "utils/LeoRand.h"

// TODO:
// PointCloud3DBuilder

namespace geo
{
	// A Point cloud of 3D points, uses glm::vec3 
	class PointCloud3D
	{
	public:
		explicit PointCloud3D(std::vector<glm::vec3> points, std::vector<glm::vec3> normals = {});
		explicit PointCloud3D(const f32* arr, index_t float_count);
	public:
		index_t Size() const;
		bool Empty() const;
		glm::vec3 Centroid() const;
	public:
		const glm::vec3& operator[](index_t i) const;
	public:
		bool HasNormals() const;
		const glm::vec3& Point(index_t i) const;
		const glm::vec3& Normal(index_t i) const;
	public:
		void Transform(const RigidTransform& transform);
		void Transform(const glm::mat3& rot, const glm::vec3& t);
	public:
		auto begin() const { return m_points.cbegin(); }
		auto end() const { return m_points.cend(); }
	public:
		const std::vector<glm::vec3>& GetStorage() const;
	private:
		void recalculateCentroid();
	private:
		std::vector<glm::vec3> m_points;
		std::vector<glm::vec3> m_normals;
		glm::vec3 m_centroid{ 0.0f };
	};

	PointCloud3D GenerateRandomPointCloudRect(const glm::vec3& center, f32 width, f32 height, f32 depth, u32 pointCount, 
		Random& rng, bool haveNormals = true);

}