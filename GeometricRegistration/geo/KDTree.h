#pragma once
#include <PointCloud3D.h>
#include <memory>

namespace geo
{
	// Kd-Tree for 3D points using glm::vec3
	class KDTree : public INearestNeighbor
	{
	public:
		explicit KDTree(const PointCloud3D& cloud);
		explicit KDTree(const std::vector<glm::vec3>& points);

		// Disable copy (tree owns internal structure)
		KDTree(const KDTree&) = delete;
		KDTree& operator=(const KDTree&) = delete;

		// Allow move
		KDTree(KDTree&&) noexcept;
		KDTree& operator=(KDTree&&) noexcept;

		~KDTree();
	public:
		size_t NearestIndex(const glm::vec3& query) const;
		virtual glm::vec3 FindClosestPoint(const glm::vec3& query) const override;
	public:
		std::vector<glm::vec3>& GetStorage();
	private:
		struct KDTreeData;
		std::unique_ptr<KDTreeData> m_data;
	};

}

