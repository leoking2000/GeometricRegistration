#include <nanoflann.hpp>
#include "KDTree.h"

using namespace nanoflann;

namespace geo
{
    struct PointCloudAdaptor
    {
        const std::vector<glm::vec3>& pts;

        PointCloudAdaptor(const std::vector<glm::vec3>& p)
            : pts(p) {
        }

        inline size_t kdtree_get_point_count() const
        {
            return pts.size();
        }

        inline float kdtree_get_pt(const size_t idx, int dim) const
        {
            if (dim == 0) return pts[idx].x;
            if (dim == 1) return pts[idx].y;
            return pts[idx].z;
        }

        template <class BBOX>
        bool kdtree_get_bbox(BBOX&) const { return false; }
    };

    struct KDTree::KDTreeData
    {
    public:
        using KDTreeType = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, PointCloudAdaptor>, PointCloudAdaptor, 3>;
    public:
        KDTreeData(const std::vector<glm::vec3>& inputPoints)
            : points(inputPoints),
            adaptor(points),
            index(3, adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10))
        {
            index.buildIndex();
        }
    public:
        std::vector<glm::vec3> points; // Maybe have a shared_ptr to a std::vector or geo::pointCloud3D to not waste memory????
        PointCloudAdaptor adaptor;
        KDTreeType index;
    };


    KDTree::KDTree(const PointCloud3D& cloud)
        :
        KDTree(cloud.GetStorage())
    {

    }

    KDTree::~KDTree() = default;

    KDTree::KDTree(KDTree&&) noexcept = default;
    KDTree& KDTree::operator=(KDTree&&) noexcept = default;

    KDTree::KDTree(const std::vector<glm::vec3>& points)
        :
        m_data(std::make_unique<KDTreeData>(points))
    {

    }

    size_t KDTree::NearestIndex(const glm::vec3& query) const
    {
        size_t resultIndex;
        float resultDistSq;

        nanoflann::KNNResultSet<float> resultSet(1);
        resultSet.init(&resultIndex, &resultDistSq);

        float queryPt[3] = { query.x, query.y, query.z };

        m_data->index.findNeighbors(resultSet, queryPt);

        return resultIndex;
    }

    glm::vec3 KDTree::FindClosestPoint(const glm::vec3& query) const
    {
        size_t idx = NearestIndex(query);
        return m_data->adaptor.pts[idx];
    }

    std::vector<glm::vec3>& KDTree::GetStorage()
    {
        return m_data->points;
    }

}