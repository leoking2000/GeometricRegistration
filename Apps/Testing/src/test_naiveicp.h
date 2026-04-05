#pragma once
#include <gtest/gtest.h>
#include <geo/GeometricRegistration.h>
#include <geo/utils/GeoRand.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/epsilon.hpp>
#include "ExpectNear.h"

static std::vector<glm::vec3> CreateFromArray(float* arr, size_t size)
{
    std::vector<glm::vec3> points;

    for (size_t i = 0; i < size; i += 3)
    {
        points.emplace_back(arr[i], arr[i + 1], arr[i + 2]);
    }

    return points;
}

TEST(NaiveICP, IdentityAlignment)
{
    float data[12] = {
        0.0f,0.0f,0.0f,
        1.0f,0.0f,0.0f,
        0.0f,1.0f,0.0f,
        0.0f,0.0f,1.0f
    };

    std::vector<glm::vec3> target_data = CreateFromArray(data, 12); 
    geo::PointCloud3D target(target_data);
    geo::PointCloud3D source(target_data);

    auto result = geo::NaiveICP(target, source, geo::LinearNN(target_data), 20);

    EXPECT_TRUE(result.converged);
    EXPECT_NEAR(result.rms, 0.0f, 1e-6f);

    ExpectMat3Near(result.transform.rotation, glm::mat3(1.0f), 1e-6f);
    ExpectVec3Near(result.transform.translation, glm::vec3(0.0f), 1e-6f);
}

TEST(NaiveICP, RecoversKnownTransform)
{
    float data[12] = {
        0.0f,0.0f,0.0f,
        1.0f,0.0f,0.0f,
        0.0f,1.0f,0.0f,
        0.0f,0.0f,1.0f
    };

    std::vector<glm::vec3> target_data = CreateFromArray(data, 12);
    geo::PointCloud3D target(target_data);
    geo::PointCloud3D source(target_data);

    float angle = glm::radians(10.0f);
    glm::mat4 rot4 = glm::rotate(glm::mat4(1.0f), angle, glm::vec3(0, 0, 1));
    glm::mat3 R = glm::mat3(rot4);

    glm::vec3 t(1.0f, 1.0f, 1.0f);

    source.Transform({ R, t });

    auto result = geo::NaiveICP(target, source, geo::LinearNN(target_data), 20);

    EXPECT_TRUE(result.converged);
    EXPECT_NEAR(result.rms, 0.0f, 1e-5f);

    glm::mat3 R_expected = glm::transpose(R);
    glm::vec3 t_expected = -R_expected * t;

    ExpectMat3Near(result.transform.rotation, R_expected, 1e-4f);
    ExpectVec3Near(result.transform.translation, t_expected, 1e-4f);
}

TEST(NaiveICP, RotationIsOrthogonal)
{
    float data[12] = {
        0.0f,0.0f,0.0f,
        1.0f,0.0f,0.0f,
        0.0f,1.0f,0.0f,
        0.0f,0.0f,1.0f
    };

    std::vector<glm::vec3> target_data = CreateFromArray(data, 12);
    geo::PointCloud3D target(target_data);
    geo::PointCloud3D source(target_data);

    auto result = geo::NaiveICP(target, source, geo::LinearNN(target_data), 20);

    glm::mat3 RtR = glm::transpose(result.transform.rotation) * result.transform.rotation;

    ExpectMat3Near(RtR, glm::mat3(1.0f), 1e-4f);
}


TEST(NaiveICP, RotationHasUnitDeterminant)
{
    float data[12] = {
        0.0f,0.0f,0.0f,
        1.0f,0.0f,0.0f,
        0.0f,1.0f,0.0f,
        0.0f,0.0f,1.0f
    };

    std::vector<glm::vec3> target_data = CreateFromArray(data, 12);
    geo::PointCloud3D target(target_data);
    geo::PointCloud3D source(target_data);

    auto result = geo::NaiveICP(target, source, geo::LinearNN(target_data), 20);

    float det = glm::determinant(result.transform.rotation);
    EXPECT_NEAR(det, 1.0f, 1e-4f);
}

TEST(NaiveICP, RMSIsReduced)
{
    float data[12] = {
        0.0f,0.0f,0.0f,
        1.0f,0.0f,0.0f,
        0.0f,1.0f,0.0f,
        0.0f,0.0f,1.0f
    };

    std::vector<glm::vec3> target_data = CreateFromArray(data, 12);
    geo::PointCloud3D target(target_data);
    geo::PointCloud3D source(target_data);

    source.Transform({ glm::mat3(1.0f), { 2.0f, 0.0f, 0.0f } });

    auto result = geo::NaiveICP(target, source, geo::LinearNN(target_data), 50);

    EXPECT_LT(result.rms, 2.0f);
}

TEST(NaiveICP, PointToPlaneRandomRect)
{
    geo::Random rng(8888); // reproducible seed

    // Generate a random "cube like" point cloud (points on the faces of a box)
    geo::PointCloud3D target = geo::GenerateRandomPointCloudRect(
        glm::vec3(0.0f), 10.0f, 10.0f, 10.0f, 100, rng, true);
    
    // Apply a small known transform
    glm::mat3 rot = glm::rotate(glm::mat4(1.0f), glm::radians(10.0f), glm::vec3(0,1,0));
    glm::vec3 trans(5.0f, 2.0f, 1.0f);

    geo::PointCloud3D source = target; // copy
    source.Transform({ rot, trans });

    // Use KDTree for speed
    geo::KDTree nn(target.GetPoints());

    // Run point-to-plane ICP
    auto result = geo::NaiveICP(target, source, nn, 100, 1e-2f, true); // useNormals = true

    // Check convergence
    EXPECT_TRUE(result.converged);

    // RMS should be very small (since we generated the transform)
    EXPECT_LT(result.rms, 1e-2f);

    // The recovered transform should be approximately the inverse of the applied transform
    glm::mat3 expectedR = glm::transpose(rot); // inverse rotation
    glm::vec3 expectedT = -expectedR * trans;  // inverse translation

    ExpectMat3Near(result.transform.rotation, expectedR, 1e-2f);
    ExpectVec3Near(result.transform.translation, expectedT, 1e-2f);
}
