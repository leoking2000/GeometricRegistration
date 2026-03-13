#include <gtest/gtest.h>
#include <glm/glm.hpp>
#include <vector>
#include <geo/utils/LeoRand.h>
#include <geo/spatial/INearestNeighbor.h>
#include <geo/spatial/LinearNN.h>
#include <geo/spatial/KDTree.h>

class KDTreeTest : public ::testing::Test
{
protected:
    std::vector<glm::vec3> points;
    std::vector<glm::vec3> rng_points;
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

        std::vector<glm::vec3> points;
        for (int i = 0; i < 10000; i++)
        {
            rng_points.emplace_back(rng.Float3(-10.0f, 10.0f));
        }
    }
};

TEST_F(KDTreeTest, ConstructionAndSize)
{
    geo::KDTree tree(points);

    EXPECT_FALSE(tree.Empty());
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

TEST_F(KDTreeTest, DistanceIsCorrect)
{
    geo::KDTree kd(points);
    geo::LinearNN linear(points);

    glm::vec3 q{ 0.2f, 0.1f, 0.0f };

    float kdDistSq;
    float linearDistSq;

    kd.Query(q, &kdDistSq);
    linear.Query(q, &linearDistSq);

    EXPECT_NEAR(kdDistSq, linearDistSq, 1e-6f);
}

TEST_F(KDTreeTest, FindClosestPoint)
{
    geo::KDTree kd(points);
    geo::LinearNN linear(points);

    glm::vec3 q{ 0.9f,0.0f,0.0f };

    auto kdPoint = kd.FindClosestPoint(q);
    auto linearPoint = points[linear.Query(q)];

    EXPECT_FLOAT_EQ(kdPoint.x, linearPoint.x);
    EXPECT_FLOAT_EQ(kdPoint.y, linearPoint.y);
    EXPECT_FLOAT_EQ(kdPoint.z, linearPoint.z);
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

    for (int i = 0; i < 5000; i++)
    {
        glm::vec3 q(rng.Float3(-10.0f, 10.0f));
        geo::index_t kdIdx = kd.Query(q);
    }
}


