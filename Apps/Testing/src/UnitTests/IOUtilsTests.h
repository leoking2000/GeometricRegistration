#pragma once
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <geo/io/IOUtils.h>

namespace fs = std::filesystem;

// ============================================================
// Helpers
// ============================================================

// Writes a string to a temp file and returns its path.
// The file is created in the system temp directory.
static fs::path WriteTempFile(const std::string& filename, const std::string& content)
{
    fs::path p = fs::temp_directory_path() / filename;
    std::ofstream f(p);
    f << content;
    return p;
}

// Writes raw bytes to a temp binary file and returns its path.
static fs::path WriteTempBinaryFile(const std::string& filename, const std::vector<geo::u8>& bytes)
{
    fs::path p = fs::temp_directory_path() / filename;
    std::ofstream f(p, std::ios::binary);
    f.write(reinterpret_cast<const char*>(bytes.data()), bytes.size());
    return p;
}

// Minimal valid OBJ: a single triangle
static const std::string kMinimalOBJ = R"(
v 0.0 0.0 0.0
v 1.0 0.0 0.0
v 0.0 1.0 0.0
vn 0.0 0.0 1.0
vn 0.0 0.0 1.0
vn 0.0 0.0 1.0
f 1//1 2//2 3//3
)";

// Minimal valid ASCII PLY: a single triangle
static const std::string kMinimalPLY_ASCII = R"(ply
format ascii 1.0
element vertex 3
property float x
property float y
property float z
property float nx
property float ny
property float nz
element face 1
property list uchar uint vertex_indices
end_header
0.0 0.0 0.0 0.0 0.0 1.0
1.0 0.0 0.0 0.0 0.0 1.0
0.0 1.0 0.0 0.0 0.0 1.0
3 0 1 2
)";

// Minimal ASCII PLY with no faces (point cloud)
static const std::string kMinimalPLY_PointCloud = R"(ply
format ascii 1.0
element vertex 3
property float x
property float y
property float z
end_header
0.0 0.0 0.0
1.0 0.0 0.0
0.0 1.0 0.0
)";

// ============================================================
// FileExists
// ============================================================

TEST(IOUtils_FileExists, ReturnsTrueForExistingFile)
{
    fs::path p = WriteTempFile("exists_test.txt", "hello");
    EXPECT_TRUE(geo::io::FileExists(p));
    fs::remove(p);
}

TEST(IOUtils_FileExists, ReturnsFalseForMissingFile)
{
    fs::path p = fs::temp_directory_path() / "definitely_does_not_exist_xyz.txt";
    EXPECT_FALSE(geo::io::FileExists(p));
}

// ============================================================
// GetFileType
// ============================================================

TEST(IOUtils_GetFileType, DetectsOBJ)
{
    EXPECT_EQ(geo::io::GetFileType("model.obj")        , geo::io::FileType::OBJ);
    EXPECT_EQ(geo::io::GetFileType("model.OBJ")        , geo::io::FileType::OBJ); // case insensitive
    EXPECT_EQ(geo::io::GetFileType("path/to/model.obj"), geo::io::FileType::OBJ);
}

TEST(IOUtils_GetFileType, DetectsPLY)
{
    EXPECT_EQ(geo::io::GetFileType("scan.ply"), geo::io::FileType::PLY);
    EXPECT_EQ(geo::io::GetFileType("scan.PLY"), geo::io::FileType::PLY);
}

TEST(IOUtils_GetFileType, ReturnsUnknownForUnsupported)
{
    EXPECT_EQ(geo::io::GetFileType("mesh.stl")   , geo::io::FileType::UNKNOWN);
    EXPECT_EQ(geo::io::GetFileType("data.fbx")   , geo::io::FileType::UNKNOWN);
    EXPECT_EQ(geo::io::GetFileType("noextension"), geo::io::FileType::UNKNOWN);
}

// ============================================================
// GetFileName
// ============================================================

TEST(IOUtils_GetFileName, ExtractsFilename)
{
    EXPECT_EQ(geo::io::GetFileName("data/meshes/bunny.obj"), "bunny.obj");
    EXPECT_EQ(geo::io::GetFileName("bunny.obj"), "bunny.obj");
}

// ============================================================
// GetParentFolder
// ============================================================

TEST(IOUtils_GetParentFolder, ExtractsParent)
{
    EXPECT_EQ(geo::io::GetParentFolder("data/meshes/bunny.obj"), fs::path("data/meshes"));
}

TEST(IOUtils_GetParentFolder, EmptyForFileWithNoDirectory)
{
    EXPECT_TRUE(geo::io::GetParentFolder("bunny.obj").empty());
}

// ============================================================
// LoadOBJ
// ============================================================

TEST(IOUtils_LoadOBJ, LoadsVerticesAndFaces)
{
    fs::path p = WriteTempFile("test.obj", kMinimalOBJ);

    geo::io::GeometryDumpData data = geo::io::LoadOBJ(p);

    EXPECT_EQ(data.points.size(), 3u);
    EXPECT_EQ(data.indexBuffer.size(), 1u);
    EXPECT_EQ(data.geometryType, geo::io::GeometryType::TRIANGLE_MESH);
    EXPECT_EQ(data.fileType, geo::io::FileType::OBJ);

    fs::remove(p);
}

TEST(IOUtils_LoadOBJ, VertexPositionsAreCorrect)
{
    fs::path p = WriteTempFile("test_pos.obj", kMinimalOBJ);
    geo::io::GeometryDumpData data = geo::io::LoadOBJ(p);

    ASSERT_EQ(data.points.size(), 3u);
    EXPECT_FLOAT_EQ(data.points[0].x, 0.0f);
    EXPECT_FLOAT_EQ(data.points[1].x, 1.0f);
    EXPECT_FLOAT_EQ(data.points[2].y, 1.0f);

    fs::remove(p);
}

TEST(IOUtils_LoadOBJ, ReturnsEmptyForMissingFile)
{
    geo::io::GeometryDumpData data = geo::io::LoadOBJ("nonexistent.obj");
    EXPECT_TRUE(data.points.empty());
    EXPECT_TRUE(data.indexBuffer.empty());
}

TEST(IOUtils_LoadOBJ, FaceIndicesAreCorrect)
{
    fs::path p = WriteTempFile("test_idx.obj", kMinimalOBJ);
    geo::io::GeometryDumpData data = geo::io::LoadOBJ(p);

    ASSERT_EQ(data.indexBuffer.size(), 1u);
    EXPECT_EQ(data.indexBuffer[0], glm::uvec3(0, 1, 2));

    fs::remove(p);
}

// ============================================================
// LoadPLY — ASCII
// ============================================================

TEST(IOUtils_LoadPLY, ASCII_LoadsVerticesAndFaces)
{
    fs::path p = WriteTempFile("test.ply", kMinimalPLY_ASCII);
    geo::io::GeometryDumpData data = geo::io::LoadPLY(p);

    EXPECT_EQ(data.points.size(), 3u);
    EXPECT_EQ(data.normals.size(), 3u);
    EXPECT_EQ(data.indexBuffer.size(), 1u);
    EXPECT_EQ(data.geometryType, geo::io::GeometryType::TRIANGLE_MESH);

    fs::remove(p);
}

TEST(IOUtils_LoadPLY, ASCII_PointCloud_NoFaces)
{
    fs::path p = WriteTempFile("test_pc.ply", kMinimalPLY_PointCloud);
    geo::io::GeometryDumpData data = geo::io::LoadPLY(p);

    EXPECT_EQ(data.points.size(), 3u);
    EXPECT_TRUE(data.indexBuffer.empty());
    EXPECT_EQ(data.geometryType, geo::io::GeometryType::POINT_CLOUD);

    fs::remove(p);
}

TEST(IOUtils_LoadPLY, ASCII_VertexPositionsAreCorrect)
{
    fs::path p = WriteTempFile("test_plypos.ply", kMinimalPLY_ASCII);
    geo::io::GeometryDumpData data = geo::io::LoadPLY(p);

    ASSERT_EQ(data.points.size(), 3u);
    EXPECT_FLOAT_EQ(data.points[0].x, 0.0f);
    EXPECT_FLOAT_EQ(data.points[1].x, 1.0f);
    EXPECT_FLOAT_EQ(data.points[2].y, 1.0f);

    fs::remove(p);
}

TEST(IOUtils_LoadPLY, ReturnsEmptyForMissingFile)
{
    geo::io::GeometryDumpData data = geo::io::LoadPLY("nonexistent.ply");
    EXPECT_TRUE(data.points.empty());
}

TEST(IOUtils_LoadPLY, ReturnsEmptyForInvalidFile)
{
    fs::path p = WriteTempFile("bad.ply", "this is not a ply file");
    geo::io::GeometryDumpData data = geo::io::LoadPLY(p);

    EXPECT_TRUE(data.points.empty());

    fs::remove(p);
}

// ============================================================
// SaveOBJ + round-trip
// ============================================================

TEST(IOUtils_SaveOBJ, WritesFile)
{
    geo::io::GeometryDumpData original;
    original.points = { {0,0,0}, {1,0,0}, {0,1,0} };
    original.indexBuffer = { {0,1,2} };

    fs::path p = fs::temp_directory_path() / "save_test.obj";
    EXPECT_TRUE(geo::io::SaveOBJ(p, original));
    EXPECT_TRUE(geo::io::FileExists(p));

    fs::remove(p);
}

TEST(IOUtils_SaveOBJ, RoundTrip_VerticesPreserved)
{
    geo::io::GeometryDumpData original;
    original.points = { {0,0,0}, {1,0,0}, {0,1,0} };
    original.indexBuffer = { {0,1,2} };

    fs::path p = fs::temp_directory_path() / "roundtrip.obj";
    ASSERT_TRUE(SaveOBJ(p, original));

    geo::io::GeometryDumpData loaded = geo::io::LoadOBJ(p);

    ASSERT_EQ(loaded.points.size(), 3u);
    EXPECT_FLOAT_EQ(loaded.points[0].x, 0.0f);
    EXPECT_FLOAT_EQ(loaded.points[1].x, 1.0f);
    EXPECT_FLOAT_EQ(loaded.points[2].y, 1.0f);

    fs::remove(p);
}

TEST(IOUtils_SaveOBJ, RoundTrip_IndicesPreserved)
{
    geo::io::GeometryDumpData original;
    original.points = { {0,0,0}, {1,0,0}, {0,1,0} };
    original.indexBuffer = { {0,1,2} };

    fs::path p = fs::temp_directory_path() / "roundtrip_idx.obj";
    ASSERT_TRUE(SaveOBJ(p, original));

    geo::io::GeometryDumpData loaded = geo::io::LoadOBJ(p);

    ASSERT_EQ(loaded.indexBuffer.size(), 1u);
    EXPECT_EQ(loaded.indexBuffer[0], glm::uvec3(0, 1, 2));

    fs::remove(p);
}

// ============================================================
// SavePLY + round-trip
// ============================================================

TEST(IOUtils_SavePLY, WritesFile)
{
    geo::io::GeometryDumpData original;
    original.points = { {0,0,0}, {1,0,0}, {0,1,0} };
    original.normals = { {0,0,1}, {0,0,1}, {0,0,1} };
    original.indexBuffer = { {0,1,2} };

    fs::path p = fs::temp_directory_path() / "save_test.ply";
    EXPECT_TRUE(SavePLY(p, original));
    EXPECT_TRUE(geo::io::FileExists(p));

    fs::remove(p);
}

TEST(IOUtils_SavePLY, RoundTrip_VerticesPreserved)
{
    geo::io::GeometryDumpData original;
    original.points = { {0,0,0}, {1,0,0}, {0,1,0} };
    original.normals = { {0,0,1}, {0,0,1}, {0,0,1} };
    original.indexBuffer = { {0,1,2} };

    fs::path p = fs::temp_directory_path() / "ply_roundtrip.ply";
    ASSERT_TRUE(SavePLY(p, original));

    geo::io::GeometryDumpData loaded = geo::io::LoadPLY(p);

    ASSERT_EQ(loaded.points.size(), 3u);
    EXPECT_FLOAT_EQ(loaded.points[0].x, 0.0f);
    EXPECT_FLOAT_EQ(loaded.points[1].x, 1.0f);
    EXPECT_FLOAT_EQ(loaded.points[2].y, 1.0f);

    fs::remove(p);
}

TEST(IOUtils_SavePLY, RoundTrip_NormalsPreserved)
{
    geo::io::GeometryDumpData original;
    original.points = { {0,0,0}, {1,0,0}, {0,1,0} };
    original.normals = { {0,0,1}, {0,0,1}, {0,0,1} };
    original.indexBuffer = { {0,1,2} };

    fs::path p = fs::temp_directory_path() / "ply_normals_rt.ply";
    ASSERT_TRUE(geo::io::SavePLY(p, original));

    geo::io::GeometryDumpData loaded = geo::io::LoadPLY(p);

    ASSERT_EQ(loaded.normals.size(), 3u);
    EXPECT_FLOAT_EQ(loaded.normals[0].z, 1.0f);

    fs::remove(p);
}

TEST(IOUtils_SavePLY, RoundTrip_IndicesPreserved)
{
    geo::io::GeometryDumpData original;
    original.points = { {0,0,0}, {1,0,0}, {0,1,0} };
    original.indexBuffer = { {0,1,2} };

    fs::path p = fs::temp_directory_path() / "ply_idx_rt.ply";
    ASSERT_TRUE(geo::io::SavePLY(p, original));

    geo::io::GeometryDumpData loaded = geo::io::LoadPLY(p);

    ASSERT_EQ(loaded.indexBuffer.size(), 1u);
    EXPECT_EQ(loaded.indexBuffer[0], glm::uvec3(0, 1, 2));

    fs::remove(p);
}

TEST(IOUtils_SavePLY, ReturnsFalseForEmptyData)
{
    geo::io::GeometryDumpData empty;
    fs::path p = fs::temp_directory_path() / "empty.ply";
    EXPECT_FALSE(geo::io::SavePLY(p, empty));
    fs::remove(p);
}

// ============================================================
// GeometryDumpData conversions
// ============================================================

TEST(IOUtils_GeometryDumpData, ToMesh_HasCorrectVertexCount)
{
    geo::io::GeometryDumpData data;
    data.points = { {0,0,0}, {1,0,0}, {0,1,0} };
    data.indexBuffer = { {0,1,2} };

    geo::Mesh mesh = data.ToMesh();
    EXPECT_EQ(mesh.VertexCount(), 3u);
    EXPECT_EQ(mesh.TriangleCount(), 1u);
}

TEST(IOUtils_GeometryDumpData, ToPointCloud_HasCorrectSize)
{
    geo::io::GeometryDumpData data;
    data.points = { {0,0,0}, {1,0,0}, {0,1,0} };
    data.normals = { {0,0,1}, {0,0,1}, {0,0,1} };

    geo::PointCloud3D cloud = data.ToPointCloud();
    EXPECT_EQ(cloud.Size(), 3u);
    EXPECT_TRUE(cloud.HasNormals());
}

