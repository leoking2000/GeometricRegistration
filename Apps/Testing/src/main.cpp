#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/epsilon.hpp>
#include <geo/GeometricRegistration.h>

static inline void ExpectMat3Near(
    const glm::mat3& A,
    const glm::mat3& B,
    float tolerance = 1e-5f)
{
    for (int c = 0; c < 3; ++c)
    {
        for (int r = 0; r < 3; ++r)
        {
            EXPECT_NEAR(A[c][r], B[c][r], tolerance) << "Mismatch at (" << r << ", " << c << ")";
        }
    }
}

static inline void ExpectVec3Near(
    const glm::vec3& a,
    const glm::vec3& b,
    float tolerance = 1e-5f)
{
    EXPECT_NEAR(a.x, b.x, tolerance);
    EXPECT_NEAR(a.y, b.y, tolerance);
    EXPECT_NEAR(a.z, b.z, tolerance);
}


// PointCloudTest

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

    cloud.Transform({ glm::mat3(transformY), t });

    glm::vec3 expected(2.0f, 4.0f, 9.0f);
    EXPECT_TRUE(glm::all(glm::epsilonEqual(cloud.Centroid(), expected, EPSILON)));
    EXPECT_TRUE(glm::all(glm::epsilonEqual(cloud.Point(0), expected, EPSILON)));
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

    cloud.Transform({ rot, t });

    glm::vec3 expectedCentroid(1.0f, 1.0f, 3.0f / 2.0f);
    EXPECT_TRUE(glm::all(glm::epsilonEqual(cloud.Centroid(), expectedCentroid, EPSILON)));
}

// RigidTransformTests

glm::mat3 RotationZ(float radians)
{
    const float c = std::cos(radians);
    const float s = std::sin(radians);

    return glm::mat3(
        glm::vec3(c, s, 0.0f),
        glm::vec3(-s, c, 0.0f),
        glm::vec3(0.0f, 0.0f, 1.0f)
    );
}

glm::mat3 RotationX(float radians)
{
    const float c = std::cos(radians);
    const float s = std::sin(radians);

    return glm::mat3(
        glm::vec3(1.0f, 0.0f, 0.0f),
        glm::vec3(0.0f, c, s),
        glm::vec3(0.0f, -s, c)
    );
}

TEST(RigidTransformTests, IdentityTransformPointReturnsSamePoint)
{
    const geo::RigidTransform T = geo::RigidTransform::Identity();
    const glm::vec3 p(1.5f, -2.0f, 3.25f);

    const glm::vec3 result = T.TransformPoint(p);

    ExpectVec3Near(result, p);
}

TEST(RigidTransformTests, IdentityTransformNormalReturnsSameNormal)
{
    const geo::RigidTransform T = geo::RigidTransform::Identity();
    const glm::vec3 n(0.0f, 1.0f, 0.0f);

    const glm::vec3 result = T.TransformNormal(n);

    ExpectVec3Near(result, n);
}

TEST(RigidTransformTests, TransformPointAppliesRotationAndTranslation)
{
    geo::RigidTransform T;
    T.rotation = RotationZ(glm::half_pi<float>());
    T.translation = glm::vec3(10.0f, 0.0f, 5.0f);

    const glm::vec3 p(1.0f, 0.0f, 0.0f);
    const glm::vec3 expected(10.0f, 1.0f, 5.0f);

    const glm::vec3 result = T.TransformPoint(p);

    ExpectVec3Near(result, expected);
}

TEST(RigidTransformTests, TransformNormalAppliesRotationOnly)
{
    geo::RigidTransform T;
    T.rotation = RotationZ(glm::half_pi<float>());
    T.translation = glm::vec3(100.0f, 200.0f, 300.0f);

    const glm::vec3 n(1.0f, 0.0f, 0.0f);
    const glm::vec3 expected(0.0f, 1.0f, 0.0f);

    const glm::vec3 result = T.TransformNormal(n);

    ExpectVec3Near(result, expected);
}

TEST(RigidTransformTests, InverseUndoesPointTransform)
{
    geo::RigidTransform T;
    T.rotation = RotationZ(0.7f);
    T.translation = glm::vec3(3.0f, -2.0f, 4.0f);

    const glm::vec3 p(1.0f, 2.0f, -1.0f);

    const glm::vec3 transformed = T.TransformPoint(p);
    const glm::vec3 recovered = T.ComputeInverse().TransformPoint(transformed);

    ExpectVec3Near(recovered, p, 1e-4f);
}

TEST(RigidTransformTests, InverseUndoesNormalTransform)
{
    geo::RigidTransform T;
    T.rotation = RotationX(0.35f);
    T.translation = glm::vec3(7.0f, 8.0f, 9.0f);

    const glm::vec3 n = glm::normalize(glm::vec3(1.0f, 2.0f, 3.0f));

    const glm::vec3 transformed = T.TransformNormal(n);
    const glm::vec3 recovered = T.ComputeInverse().TransformNormal(transformed);

    ExpectVec3Near(recovered, n, 1e-4f);
}

TEST(RigidTransformTests, ComposeMatchesSequentialApplication)
{
    geo::RigidTransform A;
    A.rotation = RotationZ(0.5f);
    A.translation = glm::vec3(1.0f, 2.0f, 3.0f);

    geo::RigidTransform B;
    B.rotation = RotationX(-0.25f);
    B.translation = glm::vec3(-4.0f, 0.5f, 2.0f);

    const geo::RigidTransform C = geo::RigidTransform::Compose(A, B);

    const glm::vec3 p(2.0f, -1.0f, 0.25f);

    const glm::vec3 sequential = A.TransformPoint(B.TransformPoint(p));
    const glm::vec3 composed = C.TransformPoint(p);

    ExpectVec3Near(composed, sequential, 1e-4f);
}

TEST(RigidTransformTests, InverseOfIdentityIsIdentity)
{
    const geo::RigidTransform I = geo::RigidTransform::Identity();
    const geo::RigidTransform inv = I.ComputeInverse();

    ExpectMat3Near(inv.rotation, glm::mat3(1.0f));
    ExpectVec3Near(inv.translation, glm::vec3(0.0f));
}

TEST(RigidTransformTests, ToMat4MatchesPointTransform)
{
    geo::RigidTransform T;
    T.rotation = RotationZ(glm::half_pi<float>());
    T.translation = glm::vec3(2.0f, 3.0f, 4.0f);

    const glm::vec3 p(1.0f, 0.0f, 5.0f);

    const glm::vec3 expected = T.TransformPoint(p);

    const glm::mat4 M = T.ToMat4();
    const glm::vec4 hp = M * glm::vec4(p, 1.0f);
    const glm::vec3 result(hp.x, hp.y, hp.z);

    ExpectVec3Near(result, expected, 1e-5f);
}

// MeshTest

class MeshTest : public ::testing::Test
{
protected:
    std::string objPath = "test_triangle.obj";

    void SetUp() override
    {
        std::ofstream file(objPath);

        file <<
            "o Triangle\n"
            "v 0 0 0\n"
            "v 1 0 0\n"
            "v 0 1 0\n"
            "vn 0 0 1\n"
            "vn 0 0 1\n"
            "vn 0 0 1\n"
            "f 1//1 2//2 3//3\n";

        file.close();
    }

    void TearDown() override
    {
        std::filesystem::remove(objPath);
    }
};

TEST_F(MeshTest, LoadOBJ)
{
    geo::Mesh mesh(objPath);

    EXPECT_GT(mesh.VertexCount(), 0);
    EXPECT_GT(mesh.TriangleCount(), 0);
}

TEST_F(MeshTest, VertexCount)
{
    geo::Mesh mesh(objPath);

    EXPECT_EQ(mesh.VertexCount(), 3);
}

TEST_F(MeshTest, TriangleCount)
{
    geo::Mesh mesh(objPath);

    EXPECT_EQ(mesh.TriangleCount(), 1);
}

TEST_F(MeshTest, BoundingBox)
{
    geo::Mesh mesh(objPath);

    glm::vec3 min = mesh.BBoxMin();
    glm::vec3 max = mesh.BBoxMax();

    EXPECT_FLOAT_EQ(min.x, 0.0f);
    EXPECT_FLOAT_EQ(min.y, 0.0f);
    EXPECT_FLOAT_EQ(min.z, 0.0f);

    EXPECT_FLOAT_EQ(max.x, 1.0f);
    EXPECT_FLOAT_EQ(max.y, 1.0f);
    EXPECT_FLOAT_EQ(max.z, 0.0f);
}

TEST_F(MeshTest, CenterComputation)
{
    geo::Mesh mesh(objPath);

    glm::vec3 center = mesh.Center();

    EXPECT_FLOAT_EQ(center.x, 0.5f);
    EXPECT_FLOAT_EQ(center.y, 0.5f);
    EXPECT_FLOAT_EQ(center.z, 0.0f);
}

TEST_F(MeshTest, TriangleNormal)
{
    geo::Mesh mesh(objPath);

    const auto& tris = mesh.Triangles();
    ASSERT_EQ(tris.size(), 1);

    glm::vec3 n = tris[0].face_normal;

    EXPECT_NEAR(n.x, 0.0f, 1e-5);
    EXPECT_NEAR(n.y, 0.0f, 1e-5);
    EXPECT_NEAR(n.z, 1.0f, 1e-5);
}

TEST_F(MeshTest, TriangleArea)
{
    geo::Mesh mesh(objPath);

    const auto& tris = mesh.Triangles();

    ASSERT_EQ(tris.size(), 1);

    EXPECT_NEAR(tris[0].area, 0.5f, 1e-5);
}

TEST_F(MeshTest, SurfaceArea)
{
    geo::Mesh mesh(objPath);

    EXPECT_NEAR(mesh.SurfaceArea(), 0.5f, 1e-5);
}

TEST_F(MeshTest, TriangleGroups)
{
    geo::Mesh mesh(objPath);

    const auto& groups = mesh.TrianglesGroups();

    ASSERT_EQ(groups.size(), 1);

    EXPECT_EQ(groups[0].length, 1);
}

TEST_F(MeshTest, VertexDataIntegrity)
{
    geo::Mesh mesh(objPath);

    const auto& verts = mesh.Vertices();

    ASSERT_EQ(verts.size(), 3);

    EXPECT_FLOAT_EQ(verts[0].x, 0.0f);
    EXPECT_FLOAT_EQ(verts[0].y, 0.0f);
    EXPECT_FLOAT_EQ(verts[0].z, 0.0f);
}

// KDTreeTest

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

    for (int i = 0; i < rng_points_large.size(); i++)
    {
        geo::index_t kdIdx = kd.Query(rng_points_large[i]);
    }
}

TEST_F(KDTreeTest, PerformanceQueryBatchTest)
{
    geo::KDTree kd(rng_points);
    std::vector<geo::index_t> indexes(rng_points_large.size(), 0);

    kd.QueryBatch(rng_points_large, indexes);
}


// ICP

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

    geo::LeastSquaresICPParameters p;
    p.maxIterations = 20;
    auto result = geo::LeastSquaresICP(target, source, geo::LinearNN(target_data), p);

    EXPECT_TRUE(result.converged);
    EXPECT_NEAR(result.rmse, 0.0f, 1e-6f);

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

    geo::LeastSquaresICPParameters p;
    p.maxIterations = 20;
    auto result = geo::LeastSquaresICP(target, source, geo::LinearNN(target_data), p);

    EXPECT_TRUE(result.converged);
    EXPECT_NEAR(result.rmse, 0.0f, 1e-5f);

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

    geo::LeastSquaresICPParameters p;
    p.maxIterations = 20;
    auto result = geo::LeastSquaresICP(target, source, geo::LinearNN(target_data), p);

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

    geo::LeastSquaresICPParameters p;
    p.maxIterations = 20;
    auto result = geo::LeastSquaresICP(target, source, geo::LinearNN(target_data), p);

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

    geo::LeastSquaresICPParameters p;
    p.maxIterations = 20;
    auto result = geo::LeastSquaresICP(target, source, geo::LinearNN(target_data), p);

    EXPECT_LT(result.rmse, 2.0f);
}

TEST(NaiveICP, PointToPlaneRandomRect)
{
    geo::Random rng(8888); // reproducible seed

    // Generate a random "cube like" point cloud (points on the faces of a box)
    geo::PointCloud3D target = geo::GenerateRandomPointCloudRect(
        glm::vec3(0.0f), 10.0f, 10.0f, 10.0f, 100, rng, true);

    // Apply a small known transform
    glm::mat3 rot = glm::rotate(glm::mat4(1.0f), glm::radians(10.0f), glm::vec3(0, 1, 0));
    glm::vec3 trans(5.0f, 2.0f, 1.0f);

    geo::PointCloud3D source = target; // copy
    source.Transform({ rot, trans });

    // Use KDTree for speed
    geo::KDTree nn(target.GetPoints());

    // Run point-to-plane ICP
    auto result = geo::LeastSquaresICP(target, source, nn, { 100, 1e-2f, true }); // useNormals = true

    // Check convergence
    EXPECT_TRUE(result.converged);

    // RMS should be very small (since we generated the transform)
    EXPECT_LT(result.rmse, 1e-2f);

    // The recovered transform should be approximately the inverse of the applied transform
    glm::mat3 expectedR = glm::transpose(rot); // inverse rotation
    glm::vec3 expectedT = -expectedR * trans;  // inverse translation

    ExpectMat3Near(result.transform.rotation, expectedR, 1e-2f);
    ExpectVec3Near(result.transform.translation, expectedT, 1e-2f);
}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	int r = RUN_ALL_TESTS();

	return r;
}