#pragma once
#include <string>
#include <vector>
#include <filesystem>
#include <unordered_map>
#include <geo/utils/GeoTypes.h>
#include <geo/math/BBox.h>

namespace geo::io
{
	// Supported geometry file formats for import/export operations.
	enum class FileType : u8
	{
		UNKNOWN,
		OBJ,
		PLY
	};

	// Infers file type (OBJ, PLY, etc.) from file extension.
	FileType GetFileType(const std::filesystem::path& path);

	// High-level classification of loaded geometry data.
	// Used to distinguish between point clouds and triangle meshes.
	enum class GeometryType : u8
	{
		UNKNOWN,
		POINT_CLOUD,
		TRIANGLE_MESH
	};

	// To store the indes to pos / normal / coords / color of the triagle
	struct TriangleIndex
	{
		glm::uvec3 vertexIndex = {};
		glm::uvec3 normalIndex = {};
		glm::uvec3 coordsIndex = {};
		glm::uvec3 colorIndex = {};
	};

	// A Triangle Group, based on the index buffer
	struct TriangleGroup
	{
		u32 start = 0;
		u32 length = 0;
		std::string material;
	};

	// Material info
	struct Material
	{
		std::string   name = "default";
		glm::vec3     base_color = { 0.95f, 0.93f, 0.89f };
		geo::f32      reflectance = 0.05f;
		geo::f32      metallic = 0.0f;
		geo::f32      roughness = 0.8f;
		std::string   texture_file_color = "";
		std::string   texture_file_normal = "";
		std::string   texture_file_mask = "";
	};


	// Intermediate container used for loading/saving geometry data.
	// Acts as a format-agnostic bridge between file IO and internal mesh/point structures.
	struct GeometryDumpData
	{
	public:
		std::filesystem::path filePath = {};
		FileType fileType = FileType::UNKNOWN;
		GeometryType geometryType = GeometryType::UNKNOWN;
	public:
		std::vector<glm::vec3> positions;
		std::vector<glm::vec3> normals;
		std::vector<glm::vec2> texcoords;
		std::vector<glm::vec3> colors;

		std::vector<TriangleIndex> indexBuffer;
		std::vector<TriangleGroup> groups;

		BBox bbox;

		bool HasNormals() const { return !normals.empty(); }
		bool HasCoords()  const { return !texcoords.empty(); }
		bool HasColors()  const { return !colors.empty(); }
		bool HasIndices() const { return !indexBuffer.empty(); }
	public:
		// Creates a String with a summery about the GeometryDumpData 
		std::string ToString() const;
	public:
		std::unordered_map<std::string, Material> materialsMap;
	};

	// Loads geometry from a file into a generic dump structure.
	// Note: LoadGeometry Computes normal when triangles exits
	GeometryDumpData LoadGeometry(const std::filesystem::path& path);

	// Saves geometry using automatic format selection (based on file extension).
	void SaveGeometry(const std::filesystem::path& path, const GeometryDumpData& data);

	struct PointCloudData
	{
		std::vector<glm::vec3> points;
		std::vector<glm::vec3> normals;
		std::vector<glm::vec3> colors;
	};

	PointCloudData MakePointCloud(const GeometryDumpData& data);

	struct VertexData
	{
		glm::vec3 point;
		glm::vec3 normal;
		glm::vec2 texCoord;
		glm::vec3 color;
	};

	struct MeshData
	{
		std::vector<VertexData> vertexBuffer;
		std::vector<u32> indexBuffer;
	};

	MeshData MakeMeshData(const GeometryDumpData& data);
}
