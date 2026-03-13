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

    void KDTree::Build()
    {
        assert(!m_data->adaptor.pts.empty());
        m_data->index.buildIndex();
    }

    index_t KDTree::Query(const glm::vec3& point, f32* distSq) const
    {
        f32 distSqBest;
        index_t best = Search(point, distSqBest);

        if (distSq != nullptr) {
            *distSq = distSqBest;
        }

        return best;
    }

    void KDTree::QueryBatch(const std::vector<glm::vec3>& points, std::vector<index_t>& results) const
    {
        if (results.size() != points.size())
        {
            results.resize(points.size());
        }

        // TODO: multithreding here
        for (size_t i = 0; i < points.size(); i++)
        {
            results[i] = Query(points[i]);
        }
    }

    bool KDTree::Empty() const
    {
        return m_data->adaptor.pts.empty();
    }

    size_t KDTree::Size() const
    {
        return m_data->adaptor.pts.size();
    }

    glm::vec3 KDTree::FindClosestPoint(const glm::vec3& query) const
    {
        f32 d;
        return m_data->adaptor.pts[Search(query, d)];
    }

    index_t KDTree::Search(const glm::vec3& query, f32& outDistSq) const
    {
        size_t idx;
        f32 distSq = 0;

        KNNResultSet<float> resultSet(1);
        resultSet.init(&idx, &distSq);

        float q[3] = { query.x, query.y, query.z };
        m_data->index.findNeighbors(resultSet, q);

        outDistSq = distSq;

        return (index_t)idx;
    }
}
