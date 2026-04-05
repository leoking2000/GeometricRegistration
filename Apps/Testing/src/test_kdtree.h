#include <gtest/gtest.h>
#include <glm/glm.hpp>
#include <vector>
#include <geo/utils/GeoRand.h>
#include <geo/spatial/INearestNeighbor.h>
#include <geo/spatial/LinearNN.h>
#include <geo/spatial/KDTree.h>

class KDTreeTest : public ::testing::Test
{
protected:
    std::vector<glm::vec3> points;
    std::vector<glm::vec3> rng_points;
    std::vector<glm::vec3> rng_points_large;
    geo::Random rng{ 8888 };

    void SetUp() override
    {
        points = {
            {0.f,0.f,0.f},
            {1.f,0.f,0.f},
            {0.f,1.f,0.f},
            {0.f,0.f,1.f},
            {1.f,1.f,1.f},
            {-1.f,-1.f,-1.f}
        };

        for (int i = 0; i < 1000; i++)
        {
            rng_points.emplace_back(rng.Float3(-20.0f, 20.0f));
        }

        for (int i = 0; i < 10000; i++)
        {
            rng_points_large.emplace_back(rng.Float3(-50.0f, 50.0f));
        }
    }
};

TEST_F(KDTreeTest, ConstructionAndSize)
{
    geo::KDTree tree(points);

    EXPECT_FALSE(tree.Size() == 0);
    EXPECT_EQ(tree.Size(), points.size());
}

TEST_F(KDTreeTest, QueryMatchesLinearNN)
{
    geo::KDTree kd(points);
    geo::LinearNN linear(points);

    std::vector<glm::vec3> queries = {
        {0.1f, 0.1f, 0.1f},
        {0.9f, 0.1f, 0.0f},
        {-0.5f,-0.5f,-0.5f},
        {0.2f, 0.8f, 0.1f}
    };

    for (const auto& q : queries)
    {
        geo::index_t kdIdx = kd.Query(q);
        geo::index_t linearIdx = linear.Query(q);

        EXPECT_EQ(kdIdx, linearIdx);
    }
}

TEST_F(KDTreeTest, BatchQueryMatchesLinear)
{
    geo::KDTree kd(points);
    geo::LinearNN linear(points);

    std::vector<glm::vec3> queries =
    {
        {0.1f,0.1f,0.1f},
        {1.1f,0.0f,0.0f},
        {-0.8f,-0.9f,-1.0f},
        {0.0f,0.9f,0.0f}
    };

    std::vector<geo::index_t> kdResults;
    std::vector<geo::index_t> linearResults;

    kd.QueryBatch(queries, kdResults);
    linear.QueryBatch(queries, linearResults);

    ASSERT_EQ(kdResults.size(), linearResults.size());

    for (size_t i = 0; i < queries.size(); i++)
    {
        EXPECT_EQ(kdResults[i], linearResults[i]);
    }
}

TEST_F(KDTreeTest, ExactPointQuery)
{
    geo::KDTree kd(points);

    for (size_t i = 0; i < points.size(); i++)
    {
        geo::index_t idx = kd.Query(points[i]);
        EXPECT_EQ(idx, i);
    }
}

TEST_F(KDTreeTest, RandomPointsMatchLinearNN)
{
    geo::KDTree kd(rng_points);
    geo::LinearNN linear(rng_points);

    for (int i = 0; i < 200; i++)
    {
        glm::vec3 q(rng.Float3(-10.0f, 10.0f));

        geo::index_t kdIdx = kd.Query(q);
        geo::index_t linearIdx = linear.Query(q);

        EXPECT_EQ(kdIdx, linearIdx);
    }
}

TEST_F(KDTreeTest, PerformanceTest)
{
    geo::KDTree kd(rng_points);

    for (int i = 0; i < 10000; i++)
    {
        glm::vec3 q(rng.Float3(-10.0f, 10.0f));
        geo::index_t kdIdx = kd.Query(q);
    }
}

TEST_F(KDTreeTest, PerformanceQueryBatchTest)
{
    geo::KDTree kd(rng_points);
    std::vector<geo::index_t> indexes(rng_points_large.size(), 0);

    kd.QueryBatch(rng_points_large, indexes);
}
