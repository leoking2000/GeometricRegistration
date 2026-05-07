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

// BBoxTests

using namespace geo;

TEST(BBoxTests, DefaultIsInvalid)
{
    geo::BBox box;
    EXPECT_FALSE(box.IsValid());
}

TEST(BBoxTests, SetAndAccessors)
{
    BBox box({ 0,0,0 }, { 2,4,6 });

    EXPECT_TRUE(box.IsValid());
    EXPECT_EQ(box.Min(), glm::vec3(0, 0, 0));
    EXPECT_EQ(box.Max(), glm::vec3(2, 4, 6));
    EXPECT_EQ(box.Size(), glm::vec3(2, 4, 6));
    EXPECT_EQ(box.Center(), glm::vec3(1, 2, 3));
    EXPECT_EQ(box.MaxSize(), 6.0f);
}

TEST(BBoxTests, SetSymmetrical)
{
    geo::BBox box;
    box.SetSymmetrical({ 1,1,1 }, { 2,2,2 });

    EXPECT_EQ(box.Min(), glm::vec3(0, 0, 0));
    EXPECT_EQ(box.Max(), glm::vec3(2, 2, 2));
}

TEST(BBoxTests, ExpandByPoint)
{
    geo::BBox box;
    box.MakeEmpty();

    box.ExpandBy({ 1,2,3 });
    EXPECT_EQ(box.Min(), glm::vec3(1, 2, 3));
    EXPECT_EQ(box.Max(), glm::vec3(1, 2, 3));

    box.ExpandBy({ -1,5,0 });
    EXPECT_EQ(box.Min(), glm::vec3(-1, 2, 0));
    EXPECT_EQ(box.Max(), glm::vec3(1, 5, 3));
}

TEST(BBoxTests, ExpandByBox)
{
    geo::BBox a({ 0,0,0 }, { 1,1,1 });
    geo::BBox b({ -2,0,0 }, { 0,2,2 });

    a.ExpandBy(b);

    EXPECT_EQ(a.Min(), glm::vec3(-2, 0, 0));
    EXPECT_EQ(a.Max(), glm::vec3(1, 2, 2));
}

TEST(BBoxTests, Contains)
{
    geo::BBox box({ 0,0,0 }, { 1,1,1 });

    EXPECT_TRUE(box.Contains({ 0.5f,0.5f,0.5f }));
    EXPECT_TRUE(box.Contains({ 0,0,0 }));
    EXPECT_TRUE(box.Contains({ 1,1,1 }));

    EXPECT_FALSE(box.Contains({ -1,0,0 }));
    EXPECT_FALSE(box.Contains({ 2,0,0 }));
}

TEST(BBoxTests, Corner)
{
    geo::BBox box({ 0,0,0 }, { 1,2,3 });

    EXPECT_EQ(box.Corner(0), glm::vec3(0, 0, 0));
    EXPECT_EQ(box.Corner(1), glm::vec3(1, 0, 0));
    EXPECT_EQ(box.Corner(2), glm::vec3(0, 2, 0));
    EXPECT_EQ(box.Corner(4), glm::vec3(0, 0, 3));
    EXPECT_EQ(box.Corner(7), glm::vec3(1, 2, 3));
}

TEST(BBoxTests, Radius)
{
    geo::BBox box({ -1,-1,-1 }, { 1,1,1 });
    EXPECT_NEAR(box.Radius(), std::sqrt(3.0f), 1e-6f);
}

TEST(BBoxTests, ExpandByFactor)
{
    geo::BBox box(glm::vec3(0, 0, 0), glm::vec3(10, 10, 10));

    box.ExpandByFactor(0.1f); // 10%

    // Expect 1 unit expansion per side
    ExpectVec3Near(box.Min(), glm::vec3(-0.5f, -0.5f, -0.5f));
    ExpectVec3Near(box.Max(), glm::vec3(10.5f, 10.5f, 10.5f));
    ExpectVec3Near(box.Size(), glm::vec3(11.0f, 11.0f, 11.0f));
}

TEST(BBoxTests, ExpandByAbsolute)
{
    geo::BBox box(glm::vec3(0, 0, 0), glm::vec3(1, 1, 1));

    box.ExpandByAbsolute(2.0f);

    ExpectVec3Near(box.Min(), glm::vec3(-2, -2, -2));
    ExpectVec3Near(box.Max(), glm::vec3(3, 3, 3));
}

TEST(BBoxTests, ExpandInvalidDoesNothing)
{
    geo::BBox box;
    box.MakeEmpty();

    box.ExpandByFactor(0.1f);

    EXPECT_FALSE(box.IsValid());
}

TEST(GridDescriptorTest, BasicCube_NoPadding)
{
    BBox bbox(glm::vec3(0, 0, 0), glm::vec3(10, 10, 10));

    geo::GridDescriptor data = geo::ComputeGridDescriptor(bbox, 10, 0.0f);

    ExpectVec3Near(data.bbox.Min(), glm::vec3(0, 0, 0));
    ExpectVec3Near(data.bbox.Max(), glm::vec3(10, 10, 10));

    EXPECT_FLOAT_EQ(data.voxelSize, 1.0f);
    EXPECT_EQ(data.gridSize, glm::uvec3(10, 10, 10));
}

TEST(GridDescriptorTest, BasicCube_WithPadding)
{
    geo::BBox bbox(glm::vec3(0, 0, 0), glm::vec3(10, 10, 10));

    geo::GridDescriptor data = geo::ComputeGridDescriptor(bbox, 10, 0.1f); // 10% padding

    // bbox expands to [-0.5, 10.5]
    ExpectVec3Near(data.bbox.Min(), glm::vec3(-0.5f, -0.5f, -0.5f));
    ExpectVec3Near(data.bbox.Max(), glm::vec3(10.5f, 10.5f, 10.5f));

    EXPECT_FLOAT_EQ(data.voxelSize, 11.0f / 10.0f);

    EXPECT_EQ(data.gridSize.x, (u32)std::ceil(11.0f / data.voxelSize));
    EXPECT_EQ(data.gridSize.y, (u32)std::ceil(11.0f / data.voxelSize));
    EXPECT_EQ(data.gridSize.z, (u32)std::ceil(11.0f / data.voxelSize));
}

TEST(GridDescriptorTest, NonUniformBox_RespectsMaxDimension)
{
    geo::BBox bbox(glm::vec3(0, 0, 0), glm::vec3(20, 5, 5));

    geo::GridDescriptor data = geo::ComputeGridDescriptor(bbox, 20, 0.0f);

    // maxDim = 20 → voxelSize = 1
    EXPECT_FLOAT_EQ(data.voxelSize, 1.0f);

    EXPECT_EQ(data.gridSize.x, 20);
    EXPECT_EQ(data.gridSize.y, 5);
    EXPECT_EQ(data.gridSize.z, 5);
}

TEST(GridDescriptorTest, GridAlwaysCoversBBox)
{
    geo::BBox bbox(glm::vec3(-2, -1, -3), glm::vec3(3, 4, 1));

    geo::GridDescriptor data = geo::ComputeGridDescriptor(bbox, 15, 0.0f);

    glm::vec3 extent = glm::vec3(data.gridSize) * data.voxelSize;

    EXPECT_GE(extent.x, data.bbox.Size().x);
    EXPECT_GE(extent.y, data.bbox.Size().y);
    EXPECT_GE(extent.z, data.bbox.Size().z);
}

// SparseVoxelGrid Tests

class SparseVoxelGridTest : public ::testing::Test
{
protected:
    GridDescriptor CreateSimpleGrid()
    {
        BBox bbox;
        bbox.Set(glm::vec3(0.0f), glm::vec3(1.0f));

        // resolution = 10 → simple grid
        return ComputeGridDescriptor(bbox, 10, 0.0f);
    }
};

TEST_F(SparseVoxelGridTest, ReturnsDefaultForUnsetVoxel)
{
    auto desc = CreateSimpleGrid();
    SparseVoxelGrid grid(desc, 123.0f);

    EXPECT_FLOAT_EQ(grid.Get({ 1, 1, 1 }), 123.0f);
}

TEST_F(SparseVoxelGridTest, SetAndGetValue)
{
    auto desc = CreateSimpleGrid();
    SparseVoxelGrid grid(desc, 0.0f);

    glm::uvec3 coord(2, 3, 4);
    grid.Set(coord, 42.0f);

    EXPECT_FLOAT_EQ(grid.Get(coord), 42.0f);
}

TEST_F(SparseVoxelGridTest, OverwritesValue)
{
    auto desc = CreateSimpleGrid();
    SparseVoxelGrid grid(desc, 0.0f);

    glm::uvec3 coord(1, 1, 1);

    grid.Set(coord, 10.0f);
    grid.Set(coord, 20.0f);

    EXPECT_FLOAT_EQ(grid.Get(coord), 20.0f);
}

TEST_F(SparseVoxelGridTest, ReturnsDefaultOutsideGrid)
{
    auto desc = CreateSimpleGrid();
    SparseVoxelGrid grid(desc, -1.0f);

    glm::uvec3 outside(desc.gridSize.x + 5, 0, 0);

    EXPECT_FLOAT_EQ(grid.Get(outside), -1.0f);
}

TEST_F(SparseVoxelGridTest, SetOutsideGridIsIgnored)
{
    auto desc = CreateSimpleGrid();
    SparseVoxelGrid grid(desc, 0.0f);

    glm::uvec3 outside(desc.gridSize.x + 1, 0, 0);

    grid.Set(outside, 99.0f);

    EXPECT_FLOAT_EQ(grid.Get(outside), 0.0f);
}

TEST_F(SparseVoxelGridTest, IsInsideWorks)
{
    auto desc = CreateSimpleGrid();
    SparseVoxelGrid grid(desc, 0.0f);

    EXPECT_TRUE(grid.IsInside({ 0, 0, 0 }));
    EXPECT_TRUE(grid.IsInside(desc.gridSize - glm::uvec3(1)));

    EXPECT_FALSE(grid.IsInside({ desc.gridSize.x, 0, 0 }));
    EXPECT_FALSE(grid.IsInside({ 0, desc.gridSize.y, 0 }));
    EXPECT_FALSE(grid.IsInside({ 0, 0, desc.gridSize.z }));
}

TEST_F(SparseVoxelGridTest, ChangeDefaultValue)
{
    auto desc = CreateSimpleGrid();
    SparseVoxelGrid grid(desc, 5.0f);

    EXPECT_FLOAT_EQ(grid.Get({ 1, 1, 1 }), 5.0f);

    grid.SetDefaultValue(10.0f);

    EXPECT_FLOAT_EQ(grid.Get({ 2, 2, 2 }), 10.0f);
}

TEST_F(SparseVoxelGridTest, MultipleVoxelStorage)
{
    auto desc = CreateSimpleGrid();
    SparseVoxelGrid grid(desc, 0.0f);

    grid.Set({ 1, 2, 3 }, 10.0f);
    grid.Set({ 4, 5, 6 }, 20.0f);
    grid.Set({ 7, 8, 9 }, 30.0f);

    EXPECT_FLOAT_EQ(grid.Get({ 1, 2, 3 }), 10.0f);
    EXPECT_FLOAT_EQ(grid.Get({ 4, 5, 6 }), 20.0f);
    EXPECT_FLOAT_EQ(grid.Get({ 7, 8, 9 }), 30.0f);
}

TEST_F(SparseVoxelGridTest, BoundaryAccess)
{
    auto desc = CreateSimpleGrid();
    SparseVoxelGrid grid(desc, 0.0f);

    glm::uvec3 maxCoord = desc.gridSize - glm::uvec3(1);

    grid.Set(maxCoord, 77.0f);

    EXPECT_FLOAT_EQ(grid.Get(maxCoord), 77.0f);
}

TEST_F(SparseVoxelGridTest, SparseBehavior)
{
    auto desc = CreateSimpleGrid();
    SparseVoxelGrid grid(desc, -5.0f);

    grid.Set({ 1, 1, 1 }, 100.0f);

    EXPECT_FLOAT_EQ(grid.Get({ 1, 1, 1 }), 100.0f);
    EXPECT_FLOAT_EQ(grid.Get({ 2, 2, 2 }), -5.0f);
}

// RMSETests

TEST(RMSETests, PointToPointRMSEReturnsF32MaxForEmptyInput)
{
    const std::vector<glm::vec3> source;
    const std::vector<glm::vec3> target;

    EXPECT_EQ(geo::PointToPointRMSE(source, target), geo::F32_MAX);
}

TEST(RMSETests, PointToPlaneRMSEReturnsF32MaxForEmptyInput)
{
    const std::vector<glm::vec3> source;
    const std::vector<glm::vec3> target;
    const std::vector<glm::vec3> normals;

    EXPECT_EQ(geo::PointToPlaneRMSE(source, target, normals), geo::F32_MAX);
}

TEST(RMSETests, PointToPointRMSEIsZeroForIdenticalInputs)
{
    const std::vector<glm::vec3> points = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 2.0f, 3.0f},
        {-1.0f, 4.0f, 0.5f}
    };

    EXPECT_FLOAT_EQ(geo::PointToPointRMSE(points, points), 0.0f);
}

TEST(RMSETests, PointToPlaneRMSEIsZeroForIdenticalInputs)
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

TEST(RMSETests, PointToPointRMSEComputesExpectedValue)
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

TEST(RMSETests, PointToPlaneRMSEUsesOnlyNormalComponent)
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

TEST(RMSETests, PointToPlaneRMSEIgnoresTangentialComponent)
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

TEST(RMSETests, LargeValuesDoNotOverflow)
{
    std::vector<glm::vec3> a = {
        {1000,1000,1000},
        {2000,2000,2000}
    };

    std::vector<glm::vec3> b = {
        {1001,1001,1001},
        {1999,1999,1999}
    };

    EXPECT_NEAR(geo::PointToPointRMSE(a, b), std::sqrt(3.0f), 1e-4f);
}

TEST(RMSETests, ConstantPlaneOffset)
{
    std::vector<glm::vec3> source = {
        {0,0,1}, {1,0,1}
    };

    std::vector<glm::vec3> target = {
        {0,0,0}, {1,0,0}
    };

    std::vector<glm::vec3> normals = {
        {0,0,1}, {0,0,1}
    };

    EXPECT_NEAR(geo::PointToPlaneRMSE(source, target, normals), 1.0f, 1e-6f);
}

TEST(RMSETests, DiagonalNormalProjection)
{
    std::vector<glm::vec3> source = {
        {1,1,0}
    };

    std::vector<glm::vec3> target = {
        {0,0,0}
    };

    std::vector<glm::vec3> normals = {
        glm::normalize(glm::vec3(1,1,0))
    };

    float rmse = geo::PointToPlaneRMSE(source, target, normals);

    float expected = std::sqrt(2.0f);

    EXPECT_NEAR(rmse, expected, 1e-5f);
}

TEST(RMSETests, AxisAlignedSimpleCase)
{
    std::vector<glm::vec3> source = { {1,0,0} };
    std::vector<glm::vec3> target = { {0,0,0} };
    std::vector<glm::vec3> normals = { {1,0,0} };

    float rmse = geo::PointToPlaneRMSE(source, target, normals);

    EXPECT_NEAR(rmse, 1.0f, 1e-6f);
}

TEST(RMSETests, SimpleCase)
{
    std::vector<glm::vec3> source = { {1,1,0} };
    std::vector<glm::vec3> target = { {0,0,0} };
    std::vector<glm::vec3> normals = { {1,0,0} };

    float rmse = geo::PointToPlaneRMSE(source, target, normals);

    EXPECT_NEAR(rmse, 1.0f, 1e-6f);
}

TEST(RMSETests, EmptyInputReturnsMax)
{
    std::vector<glm::vec3> a, b, n;

    EXPECT_EQ(geo::PointToPlaneRMSE(a, b, n), geo::F32_MAX);
}

TEST(RMSETests, MismatchedSizesReturnMax)
{
    std::vector<glm::vec3> a = { {1,2,3} };
    std::vector<glm::vec3> b = { {1,2,3}, {4,5,6} };
    std::vector<glm::vec3> n = { {0,0,1} };

    EXPECT_EQ(geo::PointToPlaneRMSE(a, b, n), geo::F32_MAX);
}

// RigidTransformTests

static inline bool RigidTransformNearlyEqual(
    const RigidTransform& A,
    const RigidTransform& B,
    f32 eps = 1e-5f)
{
    for (int c = 0; c < 3; ++c)
    {
        for (int r = 0; r < 3; ++r)
        {
            if (std::abs(A.rotation[c][r] - B.rotation[c][r]) > eps)
            {
                return false;
            }
        }
    }

    for (int i = 0; i < 3; ++i)
    {
        if (std::abs(A.translation[i] - B.translation[i]) > eps)
        {
            return false;
        }
    }

    return true;
}

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

TEST(RigidTransformTests, IdentityDoesNotChangePoint)
{
    geo::RigidTransform I = geo::RigidTransform::Identity();

    glm::vec3 p(1.0f, 2.0f, 3.0f);
    EXPECT_EQ(I.TransformPoint(p), p);
}

TEST(RigidTransformTests, InverseCancelsTransform)
{
    geo::RigidTransform T;
    T.rotation = glm::mat3(1.0f);
    T.translation = glm::vec3(1.0f, 2.0f, 3.0f);

    auto inv = T.ComputeInverse();

    glm::vec3 p(0.5f, -1.0f, 2.0f);
    glm::vec3 q = T.TransformPoint(p);
    glm::vec3 p2 = inv.TransformPoint(q);

    EXPECT_TRUE(RigidTransformNearlyEqual(
        geo::RigidTransform{ glm::mat3(1.0f), p },
        geo::RigidTransform{ glm::mat3(1.0f), p2 }
    ));
}

TEST(RigidTransformTests, CompositionIsConsistent)
{
    geo::RigidTransform A, B;

    A.translation = glm::vec3(1, 0, 0);
    B.translation = glm::vec3(0, 1, 0);

    auto C = geo::RigidTransform::Compose(A, B);

    glm::vec3 p(1, 1, 1);
    glm::vec3 direct = A.TransformPoint(B.TransformPoint(p));
    glm::vec3 composed = C.TransformPoint(p);

    EXPECT_NEAR(direct.x, composed.x, 1e-5f);
    EXPECT_NEAR(direct.y, composed.y, 1e-5f);
    EXPECT_NEAR(direct.z, composed.z, 1e-5f);
}

// SVD

TEST(SVDTest, ReconstructsMatrix)
{
    glm::mat3 A(
        1.0f, 2.0f, 3.0f,
        0.0f, 1.0f, 4.0f,
        5.0f, 6.0f, 0.0f
    );

    geo::SVDResult svd = geo::SVD(A);

    glm::mat3 U = svd.U;
    glm::mat3 V = svd.V;
    glm::mat3 S(0.0f);
    S[0][0] = svd.S.x;
    S[1][1] = svd.S.y;
    S[2][2] = svd.S.z;

    glm::mat3 reconstructed = U * S * glm::transpose(V);

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j)
        {
            EXPECT_NEAR(A[i][j], reconstructed[i][j], 1e-4f);
        }
    }
}

// SolveRigidPointToPointTest

TEST(SolveRigidPointToPointTest, Identity)
{
    std::vector<glm::vec3> pts = {
        {0,0,0}, {1,0,0}, {0,1,0}
    };

    auto T = geo::SolveRigidPointToPoint(pts, pts);

    EXPECT_NEAR(glm::determinant(T.rotation), 1.0f, 1e-5f);
    ExpectMat3Near(T.rotation, glm::mat3(1.0f), 1e-5f);

    ExpectVec3Near(T.translation, glm::vec3(0.0f), 1e-5f);
}

TEST(SolveRigidPointToPointTest, PureTranslation)
{
    std::vector<glm::vec3> src = {
        {0,0,0}, {1,0,0}, {0,1,0}
    };

    glm::vec3 t(1, 2, 3);

    std::vector<glm::vec3> trg;
    for (auto& p : src)
        trg.push_back(p + t);

    auto T = geo::SolveRigidPointToPoint(src, trg);

    EXPECT_NEAR(glm::determinant(T.rotation), 1.0f, 1e-5f);
    ExpectMat3Near(T.rotation, glm::mat3(1.0f), 1e-5f);

    ExpectVec3Near(T.translation, t, 1e-5f);
}

TEST(SolveRigidPointToPointTest, PureRotation)
{
    std::vector<glm::vec3> src = {
        {1,0,0}, {0,1,0}, {0,0,1}
    };

    float angle = glm::radians(90.0f);
    glm::mat3 R = glm::mat3(
        cos(angle), -sin(angle), 0,
        sin(angle), cos(angle), 0,
        0, 0, 1
    );

    std::vector<glm::vec3> trg;
    for (auto& p : src)
        trg.push_back(R * p);

    auto T = geo::SolveRigidPointToPoint(src, trg);

    EXPECT_NEAR(glm::determinant(T.rotation), 1.0f, 1e-5f);
    ExpectMat3Near(T.rotation, R, 1e-5f);

    ExpectVec3Near(T.translation, glm::vec3(0.0f), 1e-5f);
}

TEST(SolveRigidPointToPointTest, RotationAndTranslation)
{
    std::vector<glm::vec3> src = {
        {1,0,0}, {0,1,0}, {0,0,1}
    };

    float angle = glm::radians(45.0f);
    glm::mat3 R = glm::mat3(
        cos(angle), -sin(angle), 0,
        sin(angle), cos(angle), 0,
        0, 0, 1
    );

    glm::vec3 t(1, 2, 3);

    std::vector<glm::vec3> trg;
    for (auto& p : src)
        trg.push_back(R * p + t);

    auto T = geo::SolveRigidPointToPoint(src, trg);

    EXPECT_NEAR(glm::determinant(T.rotation), 1.0f, 1e-5f);
    ExpectMat3Near(T.rotation, R, 1e-5f);

    ExpectVec3Near(T.translation, t, 1e-5f);
}

TEST(SolveRigidPointToPointTest, ReflectionIsCorrected)
{
    std::vector<glm::vec3> src = {
        {1,0,0}, {0,1,0}, {0,0,1}
    };

    // Reflect across X
    std::vector<glm::vec3> trg = {
        {-1,0,0}, {0,1,0}, {0,0,1}
    };

    auto T = geo::SolveRigidPointToPoint(src, trg);

    // Must still be a proper rotation
    EXPECT_NEAR(glm::determinant(T.rotation), 1.0f, 1e-5f);
}

// SolveRigidPointToPlaneTest

TEST(SolveRigidPointToPlaneTest, Identity)
{
    std::vector<glm::vec3> pts = {
        {0,0,0}, {1,0,0}, {0,1,0}
    };

    std::vector<glm::vec3> normals = {
        {0,0,1}, {0,0,1}, {0,0,1}
    };

    auto T = geo::SolveRigidPointToPlane(pts, pts, normals);

    EXPECT_NEAR(glm::determinant(T.rotation), 1.0f, 1e-5f);
    ExpectMat3Near(T.rotation, glm::mat3(1.0f), 1e-5f);

    ExpectVec3Near(T.translation, glm::vec3(0.0f), 1e-5f);
}

TEST(SolveRigidPointToPlaneTest, TranslationAlongNormal)
{
    std::vector<glm::vec3> src = {
        {0,0,1}, {1,0,1}, {0,1,1}
    };

    std::vector<glm::vec3> trg = {
        {0,0,0}, {1,0,0}, {0,1,0}
    };

    std::vector<glm::vec3> normals = {
        {0,0,1}, {0,0,1}, {0,0,1}
    };

    auto T = geo::SolveRigidPointToPlane(src, trg, normals);

    EXPECT_NEAR(glm::determinant(T.rotation), 1.0f, 1e-5f);
    ExpectMat3Near(T.rotation, glm::mat3(1.0f), 1e-5f);

    ExpectVec3Near(T.translation, glm::vec3(0.0f, 0.0f, -1.0f), 1e-5f);
}

TEST(SolveRigidPointToPlaneTest, NoTangentialCorrection)
{
    std::vector<glm::vec3> src = {
        {1,0,0}, {2,0,0}, {3,0,0}
    };

    std::vector<glm::vec3> trg = {
        {2,0,0}, {3,0,0}, {4,0,0}
    };

    std::vector<glm::vec3> normals = {
        {0,1,0}, {0,1,0}, {0,1,0}
    };

    auto T = geo::SolveRigidPointToPlane(src, trg, normals);

    EXPECT_NEAR(glm::determinant(T.rotation), 1.0f, 1e-5f);
    ExpectMat3Near(T.rotation, glm::mat3(1.0f), 1e-5f);

    ExpectVec3Near(T.translation, glm::vec3(0.0f), 1e-5f);
}

TEST(SolveRigidPointToPlaneTest, SmallRotationObservable)
{
    float angle = glm::radians(15.0f);
    glm::mat4 rot4 = glm::rotate(glm::mat4(1.0f), angle, glm::vec3(1, 0, 0));
    glm::mat3 R = glm::mat3(rot4);

    std::vector<glm::vec3> trg = {
        {1,0,0}, {-2,0,4}, {5,0,-6}, {2,0,0}
    };

    std::vector<glm::vec3> normals = {
        {0,1,0}, {0,1,0}, {0,1,0}, {1,0,0}
    };

    std::vector<glm::vec3> src;
    for (auto& p : trg)
        src.push_back(R * p);

    auto T = geo::SolveRigidPointToPlane(src, trg, normals);

    EXPECT_NEAR(glm::determinant(T.rotation), 1.0f, 1e-5f);
    ExpectMat3Near(T.rotation, glm::transpose(R), 1e-2f);

    ExpectVec3Near(T.translation, glm::vec3(0.0f), 1e-5f);
}

// SolveRigidPointToPlaneShiftedTest

TEST(SolveRigidPointToPlaneShiftedTest, ZeroOffsetsMatchesOriginal)
{
    std::vector<glm::vec3> src = {
        {0,0,1}, {1,0,1}, {0,1,1}
    };

    std::vector<glm::vec3> trg = {
        {0,0,0}, {1,0,0}, {0,1,0}
    };

    std::vector<glm::vec3> normals = {
        {0,0,1}, {0,0,1}, {0,0,1}
    };

    std::vector<geo::f32> offsets(src.size(), 0.0f);

    auto T1 = geo::SolveRigidPointToPlane(src, trg, normals);
    auto T2 = geo::SolveRigidPointToPlaneShifted(src, trg, normals, offsets);

    ExpectMat3Near(T1.rotation, T2.rotation, 1e-5f);
    ExpectVec3Near(T1.translation, T2.translation, 1e-5f);
}

TEST(SolveRigidPointToPlaneShiftedTest, OffsetMovesAlongNormal)
{
    std::vector<glm::vec3> src = {
        {0,0,1}, {1,0,1}, {0,1,1}
    };

    std::vector<glm::vec3> trg = {
        {0,0,0}, {1,0,0}, {0,1,0}
    };

    std::vector<glm::vec3> normals = {
        {0,0,1}, {0,0,1}, {0,0,1}
    };

    std::vector<geo::f32> offsets(src.size(), 1.0f);

    auto T = geo::SolveRigidPointToPlaneShifted(src, trg, normals, offsets);

    // Expect translation shifted along + normal direction
    EXPECT_NEAR(T.translation.z, 0.0f, 1e-4f);
}

TEST(SolveRigidPointToPlaneShiftedTest, ZeroSystem)
{
    std::vector<glm::vec3> pts = {
        {0,0,0}, {1,0,0}, {0,1,0}
    };

    std::vector<glm::vec3> normals = {
        {0,0,1}, {0,0,1}, {0,0,1}
    };

    std::vector<geo::f32> offsets(pts.size(), 0.0f);

    auto T = geo::SolveRigidPointToPlaneShifted(pts, pts, normals, offsets);

    ExpectMat3Near(T.rotation, glm::mat3(1.0f), 1e-5f);
    ExpectVec3Near(T.translation, glm::vec3(0.0f), 1e-5f);
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

TEST(PointCloudTest, ComputeBoundingBox_Simple)
{
    std::vector<glm::vec3> pts = {
        {0,0,0},
        {1,2,3},
        {-1,5,2}
    };

    geo::PointCloud3D pc(pts);
    geo::BBox box = pc.ComputeBoundingBox();

    ExpectVec3Near(box.Min(), glm::vec3(-1, 0, 0));
    ExpectVec3Near(box.Max(), glm::vec3(1, 5, 3));
}

TEST(PointCloudTest, ComputeBoundingBox_SinglePoint)
{
    std::vector<glm::vec3> pts = {
        {2,3,4}
    };

    geo::PointCloud3D pc(pts);
    geo::BBox box = pc.ComputeBoundingBox();

    ExpectVec3Near(box.Min(), glm::vec3(2, 3, 4));
    ExpectVec3Near(box.Max(), glm::vec3(2, 3, 4));
    ExpectVec3Near(box.Size(), glm::vec3(0, 0, 0));
}

TEST(PointCloudTest, ComputeBoundingBox_NegativeCoordinates)
{
    std::vector<glm::vec3> pts = {
        {-5,-2,-1},
        {-1,-3,-4},
        {-2,-1,-6}
    };

    geo::PointCloud3D pc(pts);
    geo::BBox box = pc.ComputeBoundingBox();

    ExpectVec3Near(box.Min(), glm::vec3(-5, -3, -6));
    ExpectVec3Near(box.Max(), glm::vec3(-1, -1, -1));
}

TEST(PointCloudTest, ComputeBoundingBox_OrderIndependence)
{
    std::vector<glm::vec3> pts1 = {
        {0,0,0}, {1,2,3}, {-1,5,2}
    };

    std::vector<glm::vec3> pts2 = {
        {-1,5,2}, {0,0,0}, {1,2,3}
    };

    geo::PointCloud3D pc1(pts1);
    geo::PointCloud3D pc2(pts2);

    geo::BBox b1 = pc1.ComputeBoundingBox();
    geo::BBox b2 = pc2.ComputeBoundingBox();

    ExpectVec3Near(b1.Min(), b2.Min());
    ExpectVec3Near(b1.Max(), b2.Max());
}

TEST(PointCloudTest, ComputeBoundingBox_CenterAndSize)
{
    std::vector<glm::vec3> pts = {
        {-1,-1,-1},
        {1,1,1}
    };

    geo::PointCloud3D pc(pts);
    geo::BBox box = pc.ComputeBoundingBox();

    ExpectVec3Near(box.Center(), glm::vec3(0, 0, 0));
    ExpectVec3Near(box.Size(), glm::vec3(2, 2, 2));
}

// MeshTest

//class MeshTest : public ::testing::Test
//{
//protected:
//    std::string objPath = "test_triangle.obj";
//
//    void SetUp() override
//    {
//        std::ofstream file(objPath);
//
//        file <<
//            "o Triangle\n"
//            "v 0 0 0\n"
//            "v 1 0 0\n"
//            "v 0 1 0\n"
//            "vn 0 0 1\n"
//            "vn 0 0 1\n"
//            "vn 0 0 1\n"
//            "f 1//1 2//2 3//3\n";
//
//        file.close();
//    }
//
//    void TearDown() override
//    {
//        std::filesystem::remove(objPath);
//    }
//};

//TEST_F(MeshTest, LoadOBJ)
//{
//    geo::Mesh mesh(objPath);
//
//    EXPECT_GT(mesh.VertexCount(), 0);
//    EXPECT_GT(mesh.TriangleCount(), 0);
//}
//
//TEST_F(MeshTest, VertexCount)
//{
//    geo::Mesh mesh(objPath);
//
//    EXPECT_EQ(mesh.VertexCount(), 3);
//}
//
//TEST_F(MeshTest, TriangleCount)
//{
//    geo::Mesh mesh(objPath);
//
//    EXPECT_EQ(mesh.TriangleCount(), 1);
//}
//
//TEST_F(MeshTest, BoundingBox)
//{
//    geo::Mesh mesh(objPath);
//
//    glm::vec3 min = mesh.BBoxMin();
//    glm::vec3 max = mesh.BBoxMax();
//
//    EXPECT_FLOAT_EQ(min.x, 0.0f);
//    EXPECT_FLOAT_EQ(min.y, 0.0f);
//    EXPECT_FLOAT_EQ(min.z, 0.0f);
//
//    EXPECT_FLOAT_EQ(max.x, 1.0f);
//    EXPECT_FLOAT_EQ(max.y, 1.0f);
//    EXPECT_FLOAT_EQ(max.z, 0.0f);
//}
//
//TEST_F(MeshTest, CenterComputation)
//{
//    geo::Mesh mesh(objPath);
//
//    glm::vec3 center = mesh.Center();
//
//    EXPECT_FLOAT_EQ(center.x, 0.5f);
//    EXPECT_FLOAT_EQ(center.y, 0.5f);
//    EXPECT_FLOAT_EQ(center.z, 0.0f);
//}
//
//TEST_F(MeshTest, TriangleNormal)
//{
//    geo::Mesh mesh(objPath);
//
//    const auto& tris = mesh.Triangles();
//    ASSERT_EQ(tris.size(), 1);
//
//    glm::vec3 n = tris[0].face_normal;
//
//    EXPECT_NEAR(n.x, 0.0f, 1e-5);
//    EXPECT_NEAR(n.y, 0.0f, 1e-5);
//    EXPECT_NEAR(n.z, 1.0f, 1e-5);
//}
//
//TEST_F(MeshTest, TriangleArea)
//{
//    geo::Mesh mesh(objPath);
//
//    const auto& tris = mesh.Triangles();
//
//    ASSERT_EQ(tris.size(), 1);
//
//    EXPECT_NEAR(tris[0].area, 0.5f, 1e-5);
//}
//
//TEST_F(MeshTest, SurfaceArea)
//{
//    geo::Mesh mesh(objPath);
//
//    EXPECT_NEAR(mesh.SurfaceArea(), 0.5f, 1e-5);
//}
//
//TEST_F(MeshTest, TriangleGroups)
//{
//    geo::Mesh mesh(objPath);
//
//    const auto& groups = mesh.TrianglesGroups();
//
//    ASSERT_EQ(groups.size(), 1);
//
//    EXPECT_EQ(groups[0].length, 1);
//}
//
//TEST_F(MeshTest, VertexDataIntegrity)
//{
//    geo::Mesh mesh(objPath);
//
//    const auto& verts = mesh.Vertices();
//
//    ASSERT_EQ(verts.size(), 3);
//
//    EXPECT_FLOAT_EQ(verts[0].x, 0.0f);
//    EXPECT_FLOAT_EQ(verts[0].y, 0.0f);
//    EXPECT_FLOAT_EQ(verts[0].z, 0.0f);
//}

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

// LeastSquaresICP

static std::vector<glm::vec3> CreateFromArray(float* arr, size_t size)
{
    std::vector<glm::vec3> points;

    for (size_t i = 0; i < size; i += 3)
    {
        points.emplace_back(arr[i], arr[i + 1], arr[i + 2]);
    }

    return points;
}

TEST(LeastSquaresICP, IdentityAlignment)
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

TEST(LeastSquaresICP, RecoversKnownTransform)
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

    ExpectMat3Near(result.transform.rotation, R_expected, 1e-5f);
    ExpectVec3Near(result.transform.translation, t_expected, 1e-5f);
}

TEST(LeastSquaresICP, RotationIsOrthogonal)
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


TEST(LeastSquaresICP, RotationHasUnitDeterminant)
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

TEST(LeastSquaresICP, RMSIsReduced)
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

TEST(LeastSquaresICP, PointToPlaneRandomRect)
{
    geo::Random rng(8888); // reproducible seed

    // Generate a random "cube like" point cloud (points on the faces of a box)
    geo::PointCloud3D target = geo::GenerateRandomPointCloudRect(
        glm::vec3(0.0f), 10.0f, 10.0f, 10.0f, 100, rng, true);

    // Apply a small known transform
    glm::mat3 rot = glm::rotate(glm::mat4(1.0f), glm::radians(15.0f), glm::vec3(0, 1, 0));
    glm::vec3 trans(8.0f, 4.0f, -2.0f);

    geo::PointCloud3D source = target; // copy
    source.Transform({ rot, trans });

    // Use KDTree for speed
    geo::KDTree nn(target.GetPoints());

    // Run point-to-plane ICP
    geo::LeastSquaresICPParameters params;
    params.useNormals = true;
    auto result = geo::LeastSquaresICP(target, source, nn, params);

    // Check convergence
    EXPECT_TRUE(result.converged);

    // RMS should be very small (since we generated the transform)
    EXPECT_LT(result.rmse, 1e-5f);

    // The recovered transform should be approximately the inverse of the applied transform
    glm::mat3 expectedR = glm::transpose(rot); // inverse rotation
    glm::vec3 expectedT = -expectedR * trans;  // inverse translation

    ExpectMat3Near(result.transform.rotation, expectedR, 1e-5f);
    ExpectVec3Near(result.transform.translation, expectedT, 1e-5f);
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
        glm::vec3(1.0f, 0.0f, 1.0f)
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

// EfficientICP Tests




int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	int r = RUN_ALL_TESTS();

	return r;
}