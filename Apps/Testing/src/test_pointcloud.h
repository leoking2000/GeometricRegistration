#pragma once
#include <gtest/gtest.h>
#include <geo/GeometricRegistration.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/epsilon.hpp>

static constexpr float EPSILON = 1e-10f;

TEST(PointCloudTest, ConstructFromVector_CountCorrect)
{
    std::vector<glm::vec3> pts = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f}
    };

    geo::PointCloud3D cloud(std::move(pts));
    EXPECT_EQ(cloud.Size(), 3);
}

TEST(PointCloudTest, ConstructFromArray_CountCorrect)
{
    float data[9] = {
        0.0f, 0.0f, 0.0f,
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f
    };

    geo::PointCloud3D cloud(data, 9);
    EXPECT_EQ(cloud.Size(), 3);
}

TEST(PointCloudTest, CentroidOfTriangle)
{
    std::vector<glm::vec3> pts = {
        {0.0f, 0.0f, 0.0f},
        {2.0f, 0.0f, 0.0f},
        {0.0f, 2.0f, 0.0f}
    };

    geo::PointCloud3D cloud(std::move(pts));

    glm::vec3 expected(2.0f / 3.0f, 2.0f / 3.0f, 0.0f);
    EXPECT_TRUE(glm::all(glm::epsilonEqual(cloud.Centroid(), expected, EPSILON)));
}

TEST(PointCloudTest, CentroidOfRandomTriangle)
{
    std::vector<glm::vec3> pts = {
        {3.0f, 4.0f, -7.0f},
        {-2.0f, 3.0f, 10.0f},
        {-5.0f, -6.0f, 1.0f}
    };

    geo::PointCloud3D cloud(std::move(pts));
    glm::vec3 c = cloud.Centroid();

    glm::vec3 expected(-4.0f / 3.0f, 1.0f / 3.0f, 4.0f / 3.0f);
    EXPECT_TRUE(glm::all(glm::epsilonEqual(cloud.Centroid(), expected, EPSILON)));
}

TEST(PointCloudTest, TransformRotationAndTranslation)
{
    std::vector<glm::vec3> pts = {
        {1.0f, 0.0f, 0.0f}
    };

    geo::PointCloud3D cloud(std::move(pts));

    const glm::mat4 transformY = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    glm::vec3 t(2.0f, 4.0f, 10.0f);

    cloud.Transform(glm::mat3(transformY), t);

    glm::vec3 expected(2.0f, 4.0f, 9.0f);
    EXPECT_TRUE(glm::all(glm::epsilonEqual(cloud.Centroid(), expected, EPSILON)));
    EXPECT_TRUE(glm::all(glm::epsilonEqual(cloud[0], expected, EPSILON)));
}

TEST(PointCloudTest, TransformUpdatesCentroid)
{
    std::vector<glm::vec3> pts = {
        {0.0f, -2.0f, 3.0f},
        {2.0f, 0.0f, 0.0f}
    };

    geo::PointCloud3D cloud(std::move(pts));

    glm::mat3 rot(1.0f);
    glm::vec3 t(0.f, 2.f, 0.f);

    cloud.Transform(rot, t);

    glm::vec3 expectedCentroid(1.0f, 1.0f, 3.0f / 2.0f);
    EXPECT_TRUE(glm::all(glm::epsilonEqual(cloud.Centroid(), expectedCentroid, EPSILON)));
}

//TEST(PointCloud3D, FindClosestPoint)
//{
//    std::vector<glm::vec3> pts = {
//        {0.0f, 0.0f, 0.0f},
//        {5.0f, 0.0f, 0.0f},
//        {10.0f, 0.0f, 0.0f}
//    };
//
//    geo::PointCloud3D cloud(std::move(pts));
//
//    glm::vec3 query(6.0f, 0.0f, 0.0f);
//    glm::vec3 closest = cloud.FindClosestPoint(query);
//
//    glm::vec3 expected(5.0f, 0.0f, 0.0f);
//    EXPECT_TRUE(glm::all(glm::epsilonEqual(closest, expected, EPSILON)));
//}

