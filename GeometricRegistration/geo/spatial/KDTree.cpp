#include <cassert>
#include <algorithm>
#include <execution>
#include <nanoflann.hpp>
#include <glm/common.hpp>
#include "KDTree.h"

using namespace nanoflann;

namespace geo
{
    // Adaptor for nanoflann to work with std::vector<glm::vec3>
    struct PointCloudAdaptor
    {
        const std::vector<glm::vec3>& points_array;

        PointCloudAdaptor(const std::vector<glm::vec3>& points) : points_array(points) {}

        inline size_t kdtree_get_point_count() const { return points_array.size(); }
        inline float kdtree_get_pt(const size_t idx, int dim) const
        {
            return points_array[idx][dim];
        }

        template <class BBOX>
        bool kdtree_get_bbox(BBOX&) const { return false; }
    };

    // KDTreeData data, stores the adaptor and the index
    struct KDTree::KDTreeData
    {
        using KDTreeType = KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<float, PointCloudAdaptor>, PointCloudAdaptor, 3>;

        const std::vector<glm::vec3>& points_array;
        PointCloudAdaptor adaptor;
        KDTreeType index;

        KDTreeData(const std::vector<glm::vec3>& points)
            : 
            points_array(points),
            adaptor(points), 
            index(3, adaptor, KDTreeSingleIndexAdaptorParams(10))
        {
            index.buildIndex();
        }
    };

    // =====================================================================

    KDTree::KDTree(const std::vector<glm::vec3>& points)
    {
        assert(!points.empty());
        m_data = std::make_unique<KDTreeData>(points);
    }

    KDTree::KDTree(KDTree&&) noexcept = default;
    KDTree& KDTree::operator=(KDTree&&) noexcept = default;
    KDTree::~KDTree() = default;

    void KDTree::Rebuild()
    {
        assert(!m_data->points_array.empty());
        m_data->index.buildIndex();
    }

    index_t KDTree::Query(const glm::vec3& point) const
    {
        assert(!m_data->points_array.empty());

        size_t idx;
        f32 distSq = 0;

        KNNResultSet<float> resultSet(1);
        resultSet.init(&idx, &distSq);

        float q[3] = { point.x, point.y, point.z };
        m_data->index.findNeighbors(resultSet, q);

        return (index_t)idx;
    }

#define PARALLEL

    void KDTree::QueryBatch(const std::vector<glm::vec3>& points, std::vector<index_t>& results) const
    {
        if (results.size() != points.size())
        {
            results.resize(points.size(), 0);
        }

        assert(!m_data->points_array.empty());

#ifdef PARALLEL
        std::for_each(std::execution::par, results.begin(), results.end(),
            [&](index_t& i_placeholder)
            {
                const index_t i = &i_placeholder - results.data();

                size_t idx = 0;
                f32 distSq = 0.0f;

                nanoflann::KNNResultSet<float> resultSet(1);
                resultSet.init(&idx, &distSq);

                const float q[3] = { points[i].x, points[i].y, points[i].z };
                m_data->index.findNeighbors(resultSet, q);

                results[i] = (index_t)idx;
            });
#else
        for (size_t i = 0; i < points.size(); i++) { results[i] = Query(points[i]); }
#endif // PARALLEL
    }

    index_t KDTree::Size() const
    {
        return (index_t)m_data->points_array.size();
    }
}
