#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <geo/geometry/Mesh.h>


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



