#pragma once
#include <vector>
#include <memory>
#include "INearestNeighbor.h"

// TODO:
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
        virtual void      Build() override;
        virtual index_t   Query(const glm::vec3& point, f32* distSq = nullptr) const override;
        virtual void      QueryBatch(const std::vector<glm::vec3>& points, std::vector<index_t>& results) const override;
        virtual bool      Empty() const override;
        virtual size_t    Size()  const override;
        virtual glm::vec3 FindClosestPoint(const glm::vec3& point) const override;
    private:
        index_t Search(const glm::vec3& query, f32& outDistSq) const;
    private:
        struct KDTreeData;
        std::unique_ptr<KDTreeData> m_data;
    };

}

