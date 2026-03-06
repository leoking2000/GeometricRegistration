#pragma once
#include <vector>
#include <memory>
#include "INearestNeighbor.h"

// TODO:
// batch queries
// parallel nearest neighbor

namespace geo
{
    // Kd-Tree for 3D points using glm::vec3, store reference to a vector!!!
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
        size_t Size() const;
        // Rebuild the KDTree after the underlying point vector changes.
        // Must be called if points are inserted/removed/moved.
        void Rebuild();
    public:
        virtual glm::vec3 FindClosestPoint(const glm::vec3& p) const override;
        virtual float DistanceFromClosest(const glm::vec3& p) const override;
    private:
        size_t NearestIndex(const glm::vec3& query, float* outDistSq) const;
    private:
        struct KDTreeData;
        std::unique_ptr<KDTreeData> m_data;
    };

}

