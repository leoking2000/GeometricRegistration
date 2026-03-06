#include <nanoflann.hpp>
#include <cassert>
#include <glm/common.hpp>
#include "KDTree.h"

using namespace nanoflann;

namespace geo
{
    // Adaptor for nanoflann to work with std::vector<glm::vec3>
    struct PointCloudAdaptor
    {
        const std::vector<glm::vec3>& pts;

        PointCloudAdaptor(const std::vector<glm::vec3>& p) : pts(p) {}

        inline size_t kdtree_get_point_count() const { return pts.size(); }
        inline float kdtree_get_pt(const size_t idx, int dim) const
        {
            if (dim == 0) return pts[idx].x;
            if (dim == 1) return pts[idx].y;
            return pts[idx].z;
        }

        template <class BBOX>
        bool kdtree_get_bbox(BBOX&) const { return false; }
    };

    // KDTreeData data, stores the adaptor and the index
    struct KDTree::KDTreeData
    {
        using KDTreeType = KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<float, PointCloudAdaptor>, PointCloudAdaptor, 3>;

        PointCloudAdaptor adaptor;
        KDTreeType index;

        KDTreeData(const std::vector<glm::vec3>& points)
            : adaptor(points), index(3, adaptor, KDTreeSingleIndexAdaptorParams(10))
        {
            index.buildIndex();
        }
    };

    // =====================================================================

    KDTree::KDTree(const std::vector<glm::vec3>& points)
        : 
        m_data(std::make_unique<KDTreeData>(points))
    {
        assert(!points.empty());
    }

    KDTree::KDTree(KDTree&&) noexcept = default;
    KDTree& KDTree::operator=(KDTree&&) noexcept = default;
    KDTree::~KDTree() = default;

    size_t KDTree::Size() const
    {
        return m_data->adaptor.pts.size();
    }

    void KDTree::Rebuild()
    {
        assert(!m_data->adaptor.pts.empty());
        m_data->index.buildIndex();
    }

    size_t KDTree::NearestIndex(const glm::vec3& query, float* outDistSq) const
    {
        size_t idx;
        float distSq;

        KNNResultSet<float> resultSet(1);
        resultSet.init(&idx, &distSq);

        float q[3] = { query.x, query.y, query.z };
        m_data->index.findNeighbors(resultSet, q);

        if (outDistSq) *outDistSq = distSq;

        return idx;
    }

    glm::vec3 KDTree::FindClosestPoint(const glm::vec3& query) const
    {
        return m_data->adaptor.pts[NearestIndex(query, nullptr)];
    }

    float KDTree::DistanceFromClosest(const glm::vec3& query) const
    {
        float distSq;
        NearestIndex(query, &distSq);
        return glm::sqrt(distSq);
    }
}
