#pragma once
#include <memory>
#include "INearestNeighbor.h"

// TODO:
// parallel nearest neighbor.

namespace geo
{
    // Kd-Tree for 3D points using glm::vec3.
    // Note: The referenced points must outlive this object.
    // Note: this class is a nanoflann Adaptor.
    class KDTree : public INearestNeighbor
    {
    public:
        explicit KDTree(const std::vector<glm::vec3>& points);
    public:
        // Disable copy
        KDTree(const KDTree&) = delete;
        KDTree& operator=(const KDTree&) = delete;

        // Allow move
        KDTree(KDTree&&) noexcept;
        KDTree& operator=(KDTree&&) noexcept;

        ~KDTree();
    public:
        // Rebuilds the index after modifying the referenced point positions.
        // Queries are invalid if the referenced points change and Rebuild() is not called.
        void      Rebuild();
        index_t   Query(const glm::vec3& point) const override;
        void      QueryBatch(const std::vector<glm::vec3>& points, std::vector<index_t>& results) const override;
        index_t   Size()  const override;
    private:
        struct KDTreeData;
        std::unique_ptr<KDTreeData> m_data;
    };

}
