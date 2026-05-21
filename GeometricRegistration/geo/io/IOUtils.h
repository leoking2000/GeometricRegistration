#pragma once
#include <string>
#include <vector>
#include <filesystem>
#include <glm/glm.hpp>
#include <geo/geometry/Mesh.h>

namespace geo::io
{
	enum class FileType : u8
	{ 
		UNKNOWN,
		OBJ,
		PLY
	};

	enum class GeometryType : u8
	{
		UNKNOWN,
		POINT_CLOUD,
		TRIANGLE_MESH
	};

	struct GeometryDumpData
	{
		GeometryType geometryType = GeometryType::UNKNOWN;
		FileType fileType = FileType::UNKNOWN;

		std::filesystem::path filePath = {};

		std::vector<glm::vec3> points;
		std::vector<glm::vec3> normals;
		std::vector<glm::uvec3> indexBuffer;

		bool HasNormals() const { return !normals.empty(); }
		bool HasIndices() const { return !indexBuffer.empty(); }

		Mesh ToMesh() const;
		PointCloud3D ToPointCloud() const;
	};

	bool FileExists(const std::filesystem::path& path);
	FileType GetFileType(const std::filesystem::path& path);
	std::string GetFileName(const std::filesystem::path& path);
	std::filesystem::path GetParentFolder(const std::filesystem::path& filePath);

	GeometryDumpData LoadOBJ(const std::filesystem::path& path);
	GeometryDumpData LoadPLY(const std::filesystem::path& path);
	GeometryDumpData LoadGeometry(const std::filesystem::path& path);

	bool SaveOBJ(const std::filesystem::path& path, const GeometryDumpData& data);
	bool SavePLY(const std::filesystem::path& path, const GeometryDumpData& data);
	void SaveGeometry(const std::filesystem::path& path, const GeometryDumpData& data);
}
