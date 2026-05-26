#pragma once
#include <string>
#include <vector>
#include <filesystem>
#include <glm/glm.hpp>
#include <geo/geometry/Mesh.h>

namespace geo::io
{
	// Supported geometry file formats for import/export operations.
	enum class FileType : u8
	{ 
		UNKNOWN,
		OBJ,
		PLY
	};

	// High-level classification of loaded geometry data.
	// Used to distinguish between point clouds and triangle meshes.
	enum class GeometryType : u8
	{
		UNKNOWN,
		POINT_CLOUD,
		TRIANGLE_MESH
	};


	// Intermediate container used for loading/saving geometry data.
	// Acts as a format-agnostic bridge between file IO and internal mesh/point structures.
	struct GeometryDumpData
	{
		GeometryType geometryType = GeometryType::UNKNOWN;
		FileType fileType = FileType::UNKNOWN;

		std::filesystem::path filePath = {};

		std::vector<glm::vec3> points;
		std::vector<glm::vec3> normals;
		std::vector<glm::uvec3> indexBuffer;

		// TODO: Add Material struct???

		bool HasNormals() const { return !normals.empty(); }
		bool HasIndices() const { return !indexBuffer.empty(); }

		Mesh ToMesh() const;
		PointCloud3D ToPointCloud() const;
	};

	// Checks whether a file exists at the given path.
	bool FileExists(const std::filesystem::path& path);

	// Infers file type (OBJ, PLY, etc.) from file extension.
	FileType GetFileType(const std::filesystem::path& path);

	// Extracts the filename portion from a full filesystem path.
	std::string GetFileName(const std::filesystem::path& path);

	// Returns the parent directory of a given file path.
	std::filesystem::path GetParentFolder(const std::filesystem::path& filePath);


	// Loads geometry from an OBJ file into a generic dump structure.
	GeometryDumpData LoadOBJ(const std::filesystem::path& path);
	// Loads geometry from a PLY file into a generic dump structure.
	GeometryDumpData LoadPLY(const std::filesystem::path& path);
	// Loads geometry from a PLY file into a generic dump structure.
	GeometryDumpData LoadGeometry(const std::filesystem::path& path);


	// Writes geometry data to an OBJ file.
	bool SaveOBJ(const std::filesystem::path& path, const GeometryDumpData& data);
	// Writes geometry data to a PLY file.
	bool SavePLY(const std::filesystem::path& path, const GeometryDumpData& data);
	// Saves geometry using automatic format selection (based on file extension).
	void SaveGeometry(const std::filesystem::path& path, const GeometryDumpData& data);
}
