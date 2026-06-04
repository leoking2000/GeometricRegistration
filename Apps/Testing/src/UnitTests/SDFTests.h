#pragma once
#include <gtest/gtest.h>
#include <geo/spatial/DistanceField.h>

// ============================================================
// Helpers
// ============================================================

// Generates a UV sphere mesh centered at origin with given radius.
// The analytical SDF is: dist(p) = length(p) - radius
// Positive outside, negative inside.
static geo::Mesh MakeSphereMesh(geo::f32 radius, geo::u32 rings, geo::u32 sectors)
{
    std::vector<glm::vec3> vertices;
    std::vector<glm::uvec3> triangles;

    // Generate vertices
    for (geo::u32 r = 0; r <= rings; r++)
    {
        geo::f32 phi = glm::pi<geo::f32>() * r / rings;
        for (geo::u32 s = 0; s <= sectors; s++)
        {
            geo::f32 theta = 2.0f * glm::pi<geo::f32>() * s / sectors;
            vertices.emplace_back(
                radius * std::sin(phi) * std::cos(theta),
                radius * std::cos(phi),
                radius * std::sin(phi) * std::sin(theta)
            );
        }
    }

    // Generate triangles
    for (geo::u32 r = 0; r < rings; r++)
    {
        for (geo::u32 s = 0; s < sectors; s++)
        {
            geo::u32 a = r * (sectors + 1) + s;
            geo::u32 b = r * (sectors + 1) + s + 1;
            geo::u32 c = (r + 1) * (sectors + 1) + s;
            geo::u32 d = (r + 1) * (sectors + 1) + s + 1;

            triangles.emplace_back(a, b, c);
            triangles.emplace_back(b, d, c);
        }
    }

    return geo::Mesh("sphere", vertices, triangles); // ComputeVertexNormals() will Compute the normals.
}

// Builds a DistanceField for a mesh, with max_dist as a multiple of cellSize
static geo::DistanceField BuildDF(const geo::Mesh& mesh, geo::u32 resolution, geo::f32 bandCells)
{
    geo::f32 cellSize = mesh.BoundingBox().MaxSize() / resolution;

    geo::DistanceFieldParameters params;
    params.bounding_box = mesh.BoundingBox();
    params.resolution = resolution;
    params.max_distance = cellSize * bandCells;

    geo::DistanceField df;
    df.Build(params, mesh);
    return df;
}

// ============================================================
// Test fixture
// ============================================================

class DistanceFieldTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        // A sphere with radius 1, centered at origin
        // Analytical SDF: dist(p) = length(p) - 1.0
        m_radius = 1.0f;
        m_mesh = MakeSphereMesh(m_radius, 32, 32);
        m_df = BuildDF(m_mesh, 64, 20.0f);

        // Tolerance: queries are approximate to within one cell size
        geo::f32 cellSize = m_mesh.BoundingBox().MaxSize() / 64.0f;
        m_tol = cellSize * 1.866f;
    }

    static inline geo::f32           m_radius;
    static inline geo::Mesh          m_mesh;
    static inline geo::DistanceField m_df;
    static inline geo::f32           m_tol;
};


// ============================================================
// 1. Surface points have distance ~ 0
// ============================================================

TEST_F(DistanceFieldTest, SurfacePointsNearZero)
{
    // Sample several points exactly on the sphere surface
    std::vector<glm::vec3> surfacePoints = {
        {  m_radius,        0.0f,        0.0f },
        { -m_radius,        0.0f,        0.0f },
        {  0.0f,       m_radius,         0.0f },
        {  0.0f,      -m_radius,         0.0f },
        {  0.0f,        0.0f,        m_radius },
        {  0.0f,        0.0f,       -m_radius },
    };

    for (const auto& p : surfacePoints)
    {
        geo::f32 dist = m_df(p);
        EXPECT_NEAR(dist, 0.0f, m_tol)
            << "Surface point " << p.x << " " << p.y << " " << p.z
            << " gave distance " << dist << ", expected ~0";
    }
}

// ============================================================
// 2. Outside points have correct positive distance
// ============================================================

TEST_F(DistanceFieldTest, OutsidePointsHavePositiveDistance)
{
    // Points outside the sphere at known offsets
    // Analytical: dist = length(p) - radius
    std::vector<glm::vec3> outsidePoints = {
        { m_radius + 0.1f, 0.0f, 0.0f },
        { 0.0f, m_radius + 0.2f, 0.0f },
        { 0.0f, 0.0f, m_radius + 0.3f },
    };

    for (const auto& p : outsidePoints)
    {
        geo::f32 dist = m_df(p);
        geo::f32 expected = glm::length(p) - m_radius;  // analytical answer

        EXPECT_GT(dist, 0.0f) << "Outside point should have positive distance";
        EXPECT_NEAR(dist, expected, m_tol) << "Distance doesn't match analytical SDF";
    }
}

// ============================================================
// 3. Inside points have correct negative distance
// ============================================================

TEST_F(DistanceFieldTest, InsidePointsHaveNegativeDistance)
{
    std::vector<glm::vec3> insidePoints = {
        { m_radius - 0.1f, 0.0f, 0.0f },
        { 0.0f, m_radius - 0.2f, 0.0f },
        { 0.0f, 0.0f, m_radius - 0.3f },
    };

    for (const auto& p : insidePoints)
    {
        geo::f32 dist = m_df(p);
        geo::f32 expected = glm::length(p) - m_radius;  // negative for inside points

        EXPECT_LT(dist, 0.0f) << "Inside point should have negative distance";
        EXPECT_NEAR(dist, expected, m_tol) << "Distance doesn't match analytical SDF";
    }
}

// ============================================================
// 4. Distance grows linearly away from the surface
// ============================================================

TEST_F(DistanceFieldTest, DistanceGrowsLinearly)
{
    // Move outward from the surface along +X and check
    // that each step increases the distance by approximately cellSize
    geo::f32 cellSize = m_mesh.BoundingBox().MaxSize() / 64.0f;

    geo::f32 prevDist = 0.0f;
    for (int step = 1; step <= 3; step++)
    {
        glm::vec3 p(m_radius + step * cellSize, 0.0f, 0.0f);
        geo::f32 dist = m_df(p);

        if (step > 1)
        {
            // Each step should increase distance by roughly cellSize
            EXPECT_NEAR(dist - prevDist, cellSize, m_tol) << "Distance should grow linearly, step " << step;
        }

        prevDist = dist;
    }
}

// ============================================================
// 5. Symmetry — same distance on both sides of the surface
// ============================================================

TEST_F(DistanceFieldTest, SignedDistanceIsSymmetric)
{
    // For a sphere, SDF(center + offset) = -SDF(center - offset) is NOT true
    // But |SDF(surface + d)| ≈ |SDF(surface - d)| IS true
    geo::f32 offset = 0.5f;

    glm::vec3 outsidePoint(m_radius + offset, 0.0f, 0.0f);
    glm::vec3 insidePoint(m_radius - offset, 0.0f, 0.0f);

    geo::f32 dOutside = m_df(outsidePoint);
    geo::f32 dInside = m_df(insidePoint);

    EXPECT_NEAR(glm::abs(dOutside), glm::abs(dInside), m_tol)
        << "Distance magnitude should be symmetric across the surface";
    EXPECT_GT(dOutside, 0.0f) << "Outside should be positive";
    EXPECT_LT(dInside, 0.0f) << "Inside should be negative";
}

// ============================================================
// 6. Points beyond max_dist return max_dist
// ============================================================

TEST_F(DistanceFieldTest, BeyondBandReturnsMaxDist)
{
    geo::f32 cellSize = m_mesh.BoundingBox().MaxSize() / 64.0f;
    geo::f32 maxDist = cellSize * 20.0f;

    // A point very far outside the sphere — well beyond the narrow band
    glm::vec3 farPoint(m_radius + maxDist * 2.0f, 0.0f, 0.0f);
    geo::f32 dist = m_df(farPoint);

    EXPECT_FLOAT_EQ(dist, maxDist) << "Point beyond narrow band should return exactly max_dist";
}

// ============================================================
// 7. Origin (center of sphere) is inside and far from surface
// ============================================================

TEST_F(DistanceFieldTest, CenterOfSphereIsInsideBand)
{
    // The sphere center is radius=1.0 from the surface.
    // If max_dist < 1.0 the center is outside the band and returns max_dist.
    // If max_dist > 1.0 the center is inside the band and returns -radius.
    geo::f32 cellSize = m_mesh.BoundingBox().MaxSize() / 64.0f;
    geo::f32 maxDist = cellSize * 20.0f;

    geo::f32 dist = m_df(glm::vec3(0.0f));

    if (maxDist > m_radius)
    {
        // Center is within the band — should return approximately -radius
        EXPECT_NEAR(dist, -m_radius, m_tol);
        EXPECT_LT(dist, 0.0f);
    }
    else
    {
        // Center is outside the band — should return max_dist
        EXPECT_FLOAT_EQ(dist, maxDist);
    }
}

// ============================================================
// 8. Query outside the grid bounding box returns max_dist
// ============================================================

TEST_F(DistanceFieldTest, OutsideGridReturnMaxDist)
{
    geo::f32 cellSize = m_mesh.BoundingBox().MaxSize() / 64.0f;
    geo::f32 maxDist = cellSize * 20.0f;

    // Way outside the grid
    glm::vec3 wayOutside(1000.0f, 1000.0f, 1000.0f);
    EXPECT_FLOAT_EQ(m_df(wayOutside), maxDist);
}

// ============================================================
// 9. closestPointToTriangle unit tests
//    These test the geometry function directly, independently of the SDF
// ============================================================

TEST(ClosestPointToTriangle, QueryAtVertexA)
{
    glm::vec3 a(0, 0, 0), b(1, 0, 0), c(0, 1, 0);
    glm::vec3 fn = glm::normalize(glm::cross(b - a, c - a));

    // Query behind vertex A — closest point is A
    glm::vec3 cp; geo::f32 dist;
    bool hit = geo::closestPointToTriangle(cp, dist, glm::vec3(-1, -1, 0), a, b, c, fn);

    ASSERT_TRUE(hit);
    EXPECT_NEAR(cp.x, 0.0f, 1e-5f);
    EXPECT_NEAR(cp.y, 0.0f, 1e-5f);
    EXPECT_NEAR(dist, glm::distance(glm::vec3(-1, -1, 0), a), 1e-5f);
}

TEST(ClosestPointToTriangle, QueryAboveFace)
{
    glm::vec3 a(0, 0, 0), b(1, 0, 0), c(0, 1, 0);
    glm::vec3 fn = glm::normalize(glm::cross(b - a, c - a));

    // Query directly above face center
    glm::vec3 query(0.25f, 0.25f, 1.0f);
    glm::vec3 cp; geo::f32 dist;
    bool hit = geo::closestPointToTriangle(cp, dist, query, a, b, c, fn);

    ASSERT_TRUE(hit);
    EXPECT_NEAR(dist, 1.0f, 1e-5f);  // height above plane is exactly 1
    EXPECT_NEAR(cp.z, 0.0f, 1e-5f);  // closest point is on the plane
}

TEST(ClosestPointToTriangle, MaxDistEarlyExit)
{
    glm::vec3 a(0, 0, 0), b(1, 0, 0), c(0, 1, 0);
    glm::vec3 fn = glm::normalize(glm::cross(b - a, c - a));

    glm::vec3 cp; geo::f32 dist;
    // Query is 2 units away but maxDist is 0.5 — should return false
    bool hit = geo::closestPointToTriangle(cp, dist, glm::vec3(0.25f, 0.25f, 2.0f), a, b, c, fn, 0.5f);
    EXPECT_FALSE(hit);
}

TEST(ClosestPointToTriangle, DegenerateTriangleReturnsFalse)
{
    // All three vertices the same — degenerate triangle
    glm::vec3 a(0, 0, 0), b(0, 0, 0), c(0, 0, 0);
    glm::vec3 fn(0, 0, 1);  // arbitrary — cross product is zero

    glm::vec3 cp; geo::f32 dist;
    bool hit = geo::closestPointToTriangle(cp, dist, glm::vec3(1, 0, 0), a, b, c, fn);
    // Should not crash — may return false due to degenerate sum check
    (void)hit;
    SUCCEED();  // just testing it doesn't crash
}

// ESATest

TEST(ESATest, RecoverKnownTransform)
{
    // 1. Load or build a mesh
    geo::Mesh mesh = MakeSphereMesh(1.0f, 32, 32);

    // 2. Build SDF on the target
    geo::DistanceFieldParameters dfParams;
    dfParams.bounding_box = mesh.BoundingBox();
    dfParams.bounding_box.ExpandByAbsolute(1.0f);

    geo::f32 cellSize = mesh.BoundingBox().MaxSize() / 64.0f;

    dfParams.resolution = 64;
    dfParams.max_distance = cellSize * 30.0f;

    geo::DistanceField df;
    df.Build(dfParams, mesh);

    // 3. Sample source points from the same mesh
    geo::Random rng(42);
    geo::PointCloud3D src = mesh.SamplePointsUniform(512, rng, false);

    // 4. Apply a known small transform to the source
    glm::vec3 knownTranslation(-1.9f, -1.25f, 1.15f);
    geo::f32  knownRotation = glm::radians(30.0f);  // small rotation around Y

    glm::mat4 Ry = glm::rotate(glm::mat4(1.0f), knownRotation, glm::vec3(0, 1, 0));
    geo::RigidTransform knownT{ glm::mat3(Ry), knownTranslation };
    src.Transform(knownT);

    // 5. Run ESA to recover the transform
    // Search space covers the known perturbation with margin
    geo::ESAParameters params;

    params.seed = 42;
    params.subDim = 1;
    params.maxIterations = 5000;

    params.searchSpace.translationMin = dfParams.bounding_box.Min();
    params.searchSpace.translationMax = dfParams.bounding_box.Max();
    //params.searchSpace.rotationMin = glm::vec3(-glm::radians(60.0f));
    //params.searchSpace.rotationMax = glm::vec3(glm::radians(60.0f));

    geo::ESAResult result = RunESA(params, [&df, &src](float* e) -> float {
       const geo::RigidTransform T = geo::ConvertToRigidTransform({ e[0], e[1], e[2], e[3], e[4], e[5] });
       const geo::f32 maxDist = df.GetMaxDist();
       
       geo::f64 cost = 0.0;
       geo::u32 count = 0;
       
       for (geo::index_t i = 0; i < src.Size(); i++)
       {
           glm::vec3 tp = T.TransformPoint(src.Point(i));
           geo::f32 d = df(tp);
       
           if (glm::abs(d) < maxDist)
           {
               cost += d * d;
               count++;
           }
       }
       
       if (count == 0) {
           return maxDist * maxDist;
       }

        return float(cost / (geo::f64)count);
    });

    // 6. The recovered transform should bring src back close to the surface
    // We verify by transforming src points with the result and checking
    // average SDF value is near zero
    geo::f32 avgDist = 0.0f;
    geo::u32 count = 0;
    for (geo::index_t i = 0; i < src.Size(); i++)
    {
        glm::vec3 tp = result.transform.TransformPoint(src.Point(i));
        geo::f32 d = df(tp);
        if (glm::abs(d) < df.GetMaxDist())
        {
            avgDist += glm::abs(d);
            count++;
        }
    }

    ASSERT_GT(count, 0u) << "No points landed inside the narrow band";
    avgDist /= count;

    // Average distance to surface after alignment should be small
    // relative to cell size — within 2 cells is a good threshold
    EXPECT_LT(avgDist, cellSize * 2.0f) << "ESA did not recover the transform - avg surface distance too large";

    // RMSE should be small
    EXPECT_LT(result.cost, cellSize * 2.0f) << "RMSE is too large";
}
