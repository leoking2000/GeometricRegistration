#include <filesystem>
#include <fstream>
#include <thread>
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

// geo::Random class Test

TEST(GeoRand, SameSeedProducesSameIntSequence)
{
    geo::Random a(12345u);
    geo::Random b(12345u);

    for (int i = 0; i < 100; ++i)
        EXPECT_EQ(a.Int(-50, 50), b.Int(-50, 50));
}

TEST(GeoRand, SameSeedProducesSameFloatSequence)
{
    geo::Random a(12345u);
    geo::Random b(12345u);

    for (int i = 0; i < 100; ++i)
        EXPECT_FLOAT_EQ(a.Float(-2.0f, 3.0f), b.Float(-2.0f, 3.0f));
}

TEST(GeoRand, SetSeedResetsSequence)
{
    geo::Random rng(2026u);

    const geo::i32 first = rng.Int(0, 1000);
    rng.Int(0, 1000);
    rng.Int(0, 1000);

    rng.SetSeed(2026u);
    EXPECT_EQ(first, rng.Int(0, 1000));
}

TEST(GeoRand, IntStaysWithinInclusiveBounds)
{
    geo::Random rng(9999u);

    for (int i = 0; i < 1000; ++i)
    {
        const geo::i32 v = rng.Int(-3, 5);
        EXPECT_GE(v, -3);
        EXPECT_LE(v, 5);
    }
}

TEST(GeoRand, UIntStaysWithinInclusiveBounds)
{
    geo::Random rng(8888u);

    for (int i = 0; i < 1000; ++i)
    {
        const geo::u32 v = rng.UInt(2u, 9u);
        EXPECT_GE(v, 2u);
        EXPECT_LE(v, 9u);
    }
}

TEST(GeoRand, FloatStaysWithinRange)
{
    geo::Random rng(7777u);

    for (int i = 0; i < 1000; ++i)
    {
        const geo::f32 v = rng.Float(-1.5f, 2.5f);
        EXPECT_GE(v, -1.5f);
        EXPECT_LT(v, 2.5f);
    }
}

TEST(GeoRand, FloatReturnsExactValueForDegenerateRange)
{
    geo::Random rng(6666u);

    for (int i = 0; i < 100; ++i)
        EXPECT_FLOAT_EQ(rng.Float(3.25f, 3.25f), 3.25f);
}

TEST(GeoRand, Float2ComponentsStayWithinRange)
{
    geo::Random rng(5555u);

    for (int i = 0; i < 1000; ++i)
    {
        const glm::vec2 v = rng.Float2(-2.0f, 4.0f);
        EXPECT_GE(v.x, -2.0f);
        EXPECT_LT(v.x, 4.0f);
        EXPECT_GE(v.y, -2.0f);
        EXPECT_LT(v.y, 4.0f);
    }
}

TEST(GeoRand, Float3ComponentsStayWithinRange)
{
    geo::Random rng(4444u);

    for (int i = 0; i < 1000; ++i)
    {
        const glm::vec3 v = rng.Float3(-2.0f, 4.0f);
        EXPECT_GE(v.x, -2.0f);
        EXPECT_LT(v.x, 4.0f);
        EXPECT_GE(v.y, -2.0f);
        EXPECT_LT(v.y, 4.0f);
        EXPECT_GE(v.z, -2.0f);
        EXPECT_LT(v.z, 4.0f);
    }
}

TEST(GeoRand, Dir2DHasRequestedLength)
{
    geo::Random rng(3333u);

    for (int i = 0; i < 1000; ++i)
    {
        const glm::vec2 v = rng.Dir2D(2.5f);
        EXPECT_NEAR(glm::length(v), 2.5f, 1e-5f);
    }
}

TEST(GeoRand, Dir3DHasRequestedLength)
{
    geo::Random rng(2222u);

    for (int i = 0; i < 1000; ++i)
    {
        const glm::vec3 v = rng.Dir3D(2.5f);
        EXPECT_NEAR(glm::length(v), 2.5f, 1e-5f);
    }
}

TEST(GeoRand, ZeroLengthDirectionsReturnZeroVector)
{
    geo::Random rng(2026u);

    EXPECT_EQ(rng.Dir2D(0.0f), glm::vec2(0.0f));
    EXPECT_EQ(rng.Dir3D(0.0f), glm::vec3(0.0f));
}

// GeoTime

TEST(GeoTime, TimeDifferenceMsReturnsPositiveDuration)
{
    const geo::TimePoint start = geo::Clock::now();
    const geo::TimePoint end = start + std::chrono::milliseconds(5);

    const geo::f64 dt = geo::TimeDifferenceMs(end, start);

    EXPECT_NEAR(dt, 5.0, 1e-6);
}

TEST(GeoTime, TimingStatIsEmptyInitially)
{
    geo::TimingStat stat;

    EXPECT_TRUE(stat.Empty());
    EXPECT_EQ(stat.count, 0u);
    EXPECT_DOUBLE_EQ(stat.totalMs, 0.0);
    EXPECT_DOUBLE_EQ(stat.AverageMs(), 0.0);
}

TEST(GeoTime, TimingStatAddSampleUpdatesTotals)
{
    geo::TimingStat stat;

    stat.AddSample(10.0);
    stat.AddSample(20.0);

    EXPECT_FALSE(stat.Empty());
    EXPECT_EQ(stat.count, 2u);
    EXPECT_DOUBLE_EQ(stat.totalMs, 30.0);
    EXPECT_DOUBLE_EQ(stat.AverageMs(), 15.0);
}

TEST(GeoTime, TimingStatClampsNegativeSampleToZero)
{
    geo::TimingStat stat;

    stat.AddSample(-5.0);

    EXPECT_EQ(stat.count, 1u);
    EXPECT_DOUBLE_EQ(stat.totalMs, 0.0);
    EXPECT_DOUBLE_EQ(stat.AverageMs(), 0.0);
}

TEST(GeoTime, TimingStatToStringForEmptyStat)
{
    geo::TimingStat stat;

    EXPECT_EQ(stat.ToString(), "count=0 total=0ms avg=0ms");
}

TEST(GeoTime, ScopedTimerAddsSampleOnDestruction)
{
    geo::TimingStat stat;

    {
        geo::ScopedTimer timer(&stat);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    EXPECT_EQ(stat.count, 1u);
    EXPECT_GE(stat.totalMs, 0.0);
}

TEST(GeoTime, StopwatchIsNotRunningInitially)
{
    geo::Stopwatch sw;

    EXPECT_FALSE(sw.IsRunning());
    EXPECT_DOUBLE_EQ(sw.ElapsedMs(), 0.0);
    EXPECT_DOUBLE_EQ(sw.StopMs(), 0.0);
}

TEST(GeoTime, StopwatchStartAndStopWorks)
{
    geo::Stopwatch sw;

    sw.Start();
    EXPECT_TRUE(sw.IsRunning());

    std::this_thread::sleep_for(std::chrono::milliseconds(2));

    const geo::f64 elapsed = sw.StopMs();

    EXPECT_FALSE(sw.IsRunning());
    EXPECT_GE(elapsed, 0.0);
    EXPECT_DOUBLE_EQ(sw.ElapsedMs(), 0.0);
}

TEST(GeoTime, StopwatchRestartStartsWhenStopped)
{
    geo::Stopwatch sw;

    const geo::f64 elapsed = sw.RestartMs();

    EXPECT_DOUBLE_EQ(elapsed, 0.0);
    EXPECT_TRUE(sw.IsRunning());
}

TEST(GeoTime, StopwatchRestartReturnsElapsedWhenRunning)
{
    geo::Stopwatch sw;
    sw.Start();

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    const geo::f64 elapsed = sw.RestartMs();

    EXPECT_TRUE(sw.IsRunning());
    EXPECT_GE(elapsed, 0.0);
}

TEST(GeoTime, StopwatchElapsedIsNonNegativeWhileRunning)
{
    geo::Stopwatch sw;
    sw.Start();

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    const geo::f64 elapsed = sw.ElapsedMs();

    EXPECT_TRUE(sw.IsRunning());
    EXPECT_GE(elapsed, 0.0);
}

// Stats Tests

TEST(Stats, PointToPointRMSEReturnsF32MaxForEmptyInput)
{
    const std::vector<glm::vec3> source;
    const std::vector<glm::vec3> target;

    EXPECT_EQ(geo::PointToPointRMSE(source, target), geo::F32_MAX);
}

TEST(Stats, PointToPlaneRMSEReturnsF32MaxForEmptyInput)
{
    const std::vector<glm::vec3> source;
    const std::vector<glm::vec3> target;
    const std::vector<glm::vec3> normals;

    EXPECT_EQ(geo::PointToPlaneRMSE(source, target, normals), geo::F32_MAX);
}

TEST(Stats, PointToPointRMSEIsZeroForIdenticalInputs)
{
    const std::vector<glm::vec3> points = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 2.0f, 3.0f},
        {-1.0f, 4.0f, 0.5f}
    };

    EXPECT_FLOAT_EQ(geo::PointToPointRMSE(points, points), 0.0f);
}

TEST(Stats, PointToPlaneRMSEIsZeroForIdenticalInputs)
{
    const std::vector<glm::vec3> source = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 2.0f, 3.0f}
    };

    const std::vector<glm::vec3> target = source;
    const std::vector<glm::vec3> normals = {
        {0.0f, 0.0f, 1.0f},
        {1.0f, 0.0f, 0.0f}
    };

    EXPECT_FLOAT_EQ(geo::PointToPlaneRMSE(source, target, normals), 0.0f);
}

TEST(Stats, PointToPointRMSEComputesExpectedValue)
{
    const std::vector<glm::vec3> source = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 2.0f, 0.0f}
    };

    const std::vector<glm::vec3> target = {
        {0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f}
    };

    // sqrt((1^2 + 2^2) / 2) = sqrt(2.5)
    const float expected = std::sqrt(2.5f);

    EXPECT_NEAR(geo::PointToPointRMSE(source, target), expected, 1e-6f);
}

TEST(Stats, PointToPlaneRMSEUsesOnlyNormalComponent)
{
    const std::vector<glm::vec3> source = {
        {1.0f, 5.0f, 0.0f},
        {2.0f, 7.0f, 0.0f}
    };

    const std::vector<glm::vec3> target = {
        {0.0f, 1.0f, 0.0f},
        {0.0f, 3.0f, 0.0f}
    };

    const std::vector<glm::vec3> normals = {
        {1.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f}
    };

    // residuals along normals: 1, 2 -> rmse = sqrt((1 + 4)/2) = sqrt(2.5)
    const float expected = std::sqrt(2.5f);

    EXPECT_NEAR(geo::PointToPlaneRMSE(source, target, normals), expected, 1e-6f);
}

TEST(Stats, PointToPlaneRMSEIgnoresTangentialComponent)
{
    const std::vector<glm::vec3> source = {
        {0.0f, 10.0f, 0.0f},
        {0.0f, -3.0f, 0.0f}
    };

    const std::vector<glm::vec3> target = {
        {0.0f, 0.0f, 0.0f},
        {0.0f, 5.0f, 0.0f}
    };

    const std::vector<glm::vec3> normals = {
        {1.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f}
    };

    EXPECT_FLOAT_EQ(geo::PointToPlaneRMSE(source, target, normals), 0.0f);
}

TEST(Stats, PointToPlaneRMSEHandlesNonUnitNormalsAsImplemented)
{
    const std::vector<glm::vec3> source = {
        {1.0f, 0.0f, 0.0f}
    };

    const std::vector<glm::vec3> target = {
        {0.0f, 0.0f, 0.0f}
    };

    const std::vector<glm::vec3> normals = {
        {2.0f, 0.0f, 0.0f}
    };

    // dot((1,0,0),(2,0,0)) = 2
    EXPECT_FLOAT_EQ(geo::PointToPlaneRMSE(source, target, normals), 2.0f);
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

static std::vector<glm::vec3> MakeRandomPoints(std::size_t count, unsigned seed)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> dist(-100.0f, 100.0f);

    std::vector<glm::vec3> points;
    points.reserve(count);

    for (std::size_t i = 0; i < count; ++i)
    {
        points.push_back({ dist(rng), dist(rng), dist(rng) });
    }

    return points;
}

static double MeasureBatchQueryMs(const geo::INearestNeighbor& nn, const std::vector<glm::vec3>& queries)
{
    std::vector<geo::index_t> results;
    const auto t0 = std::chrono::steady_clock::now();
    nn.QueryBatch(queries, results);
    const auto t1 = std::chrono::steady_clock::now();

    return std::chrono::duration<double, std::milli>(t1 - t0).count();
}

TEST(KDTreePerformance, KDTreeBatchQueryScalesToLargeInput)
{
    const std::vector<glm::vec3> points = MakeRandomPoints(200000, 11u);
    const std::vector<glm::vec3> queries = MakeRandomPoints(50000, 22u);

    geo::KDTree tree(points);

    const double kdMs = MeasureBatchQueryMs(tree, queries);

    std::cout << "\nKDTree large batch query time: " << kdMs << " ms\n";

    EXPECT_GT(kdMs, 0.0);
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

// SparseICP Tests

TEST(SparseICPTest, ShrinkLpReturnsZeroForZeroInput)
{
    const glm::vec3 h(0.0f, 0.0f, 0.0f);
    const geo::f32 p = 0.5f;
    const geo::f32 mu = 10.0f;

    const glm::vec3 z = geo::ShrinkLp(h, p, mu);

    ExpectVec3Near(z, glm::vec3(0.0f), 1e-7f);
}

TEST(SparseICPTest, ShrinkLpScalarReturnsZeroForZeroInput)
{
    const geo::f32 h = 0.0f;
    const geo::f32 p = 0.5f;
    const geo::f32 mu = 10.0f;

    const geo::f32 z = geo::ShrinkLpScalar(h, p, mu);

    EXPECT_NEAR(z, 0.0f, 1e-7f);
}

TEST(SparseICPTest, ShrinkLpDoesNotIncreaseMagnitude)
{
    const glm::vec3 h(3.0f, -4.0f, 0.0f); // norm = 5
    const geo::f32 p = 0.5f;
    const geo::f32 mu = 10.0f;

    const glm::vec3 z = geo::ShrinkLp(h, p, mu);

    EXPECT_LE(glm::length(z), glm::length(h) + 1e-6f);
}

TEST(SparseICPTest, ShrinkLpPreservesDirectionWhenNonzero)
{
    const glm::vec3 h(2.0f, -1.0f, 4.0f);
    const geo::f32 p = 0.5f;
    const geo::f32 mu = 10.0f;

    const glm::vec3 z = geo::ShrinkLp(h, p, mu);

    if (glm::length(z) > 1e-7f)
    {
        const glm::vec3 h_dir = glm::normalize(h);
        const glm::vec3 z_dir = glm::normalize(z);
        ExpectVec3Near(h_dir, z_dir, 1e-5f);
    }
}

TEST(SparseICPTest, ShrinkLpScalarPreservesSign)
{
    const geo::f32 p = 0.5f;
    const geo::f32 mu = 10.0f;

    const geo::f32 pos = geo::ShrinkLpScalar(3.0f, p, mu);
    const geo::f32 neg = geo::ShrinkLpScalar(-3.0f, p, mu);

    EXPECT_GE(pos, 0.0f);
    EXPECT_LE(neg, 0.0f);
}

TEST(SparseICPTest, SparsePointToPointConvergesOnIdenticalClouds)
{
    std::vector<glm::vec3> points({
        glm::vec3(0.0f, 0.0f, 0.0f),
        glm::vec3(1.0f, 0.0f, 0.0f),
        glm::vec3(0.0f, 1.0f, 0.0f),
        glm::vec3(0.0f, 0.0f, 1.0f)
        });

    geo::PointCloud3D target(std::move(points));
    // Apply a small known transform
    glm::mat3 rot = glm::rotate(glm::mat4(1.0f), glm::radians(5.0f), glm::vec3(0, 1, 0));
    glm::vec3 trans(1.0f, 0.2f, 0.1f);

    geo::PointCloud3D source = target; // copy
    source.Transform({ rot, trans });

    geo::LinearNN nn(target.GetPoints());

    geo::SparseICPParameters params;
    params.maxIterations = 200;
    params.admmIterations = 10;
    params.tolerance = 1e-6f;
    params.p = 0.9f;
    params.mu = 10.0f;

    geo::ICPResult result = geo::SparseICPPointToPoint(target, source, nn, params);

    EXPECT_TRUE(result.converged);
    EXPECT_NEAR(result.rmse, 0.0f, 1e-5f);
    //EXPECT_LT(result.rmse, 1e-3f);
}

TEST(SparseICPTest, SparsePointToPlaneConvergesOnIdenticalCloudsWithNormals)
{
    std::vector<glm::vec3> points({
        glm::vec3(0.0f, 0.0f, 0.0f),
        glm::vec3(1.0f, 0.0f, 0.0f),
        glm::vec3(0.0f, 1.0f, 0.0f),
        glm::vec3(1.0f, 0.0f, 1.0f)
        });

    std::vector<glm::vec3> normals = {
        glm::vec3(0.0f, 0.0f, 1.0f),
        glm::vec3(0.0f, 0.0f, 1.0f),
        glm::vec3(0.0f, 0.0f, 1.0f),
        glm::vec3(0.0f, 0.0f, 1.0f)
    };

    geo::PointCloud3D target(std::move(points), std::move(normals));
    // Apply a small known transform
    glm::mat3 rot = glm::rotate(glm::mat4(1.0f), glm::radians(5.0f), glm::vec3(0, 1, 0));
    glm::vec3 trans(1.0f, 0.2f, 0.1f);

    geo::PointCloud3D source = target; // copy
    source.Transform({ rot, trans });

    geo::LinearNN nn(target.GetPoints());

    geo::SparseICPParameters params;
    params.maxIterations = 200;
    params.admmIterations = 10;
    params.tolerance = 1e-6f;
    params.p = 0.9f;
    params.mu = 10.0f;

    geo::ICPResult result = geo::SparseICPPointToPlane(target, source, nn, params);

    EXPECT_TRUE(result.converged);
    EXPECT_NEAR(result.rmse, 0.0f, 1e-5f);
    //EXPECT_LT(result.rmse, 1e-3f);
}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	int r = RUN_ALL_TESTS();

	return r;
}