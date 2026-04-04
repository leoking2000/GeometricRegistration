#pragma once
#include <vector>
#include <map>
#include <string>
#include <filesystem>
#include <glm/glm.hpp>
#include "utils//GeoTypes.h"

namespace geo
{
	struct Triangle
	{
		index_t vertex[3] = { 0,0,0 };
		index_t normal[3] = { 0,0,0 };
		index_t coords[3] = { 0,0,0 };
		f32 area          = 0.0f;
		glm::vec3 face_normal{ 0.0f };
	};

	struct TriangleGroup
	{
		index_t     start  = 0;
		index_t     length = 0;
		std::string matname;
	};

	struct Material
	{
		std::string name = "default";

		glm::vec3 baseColor = { 0.95f, 0.93f, 0.89f };

		f32 metallic    = 0.0f;
		f32 roughness   = 0.8f;
		f32 reflectance = 0.05f;

		std::string textureColor;
		std::string textureNormal;
		std::string textureMask;

		u32 texIdColor  = 0;
		u32 texIdNormal = 0;
		u32 texIdMask   = 0;
	};

	class Mesh
	{
	public:
		Mesh(std::filesystem::path filepath);
	public:
		const std::string&                     FileName()        const { return m_filename; }
		const std::vector<glm::vec3>&          Vertices()        const { return m_vertices_buffer; }
		const std::vector<glm::vec3>&          Normals()         const { return m_normals_buffer; }
		const std::vector<glm::vec3>&          Colors()          const { return m_colors_buffer; }
		const std::vector<glm::vec3>&          Coords()          const { return m_coords_buffer; } // 3rd coord is the gid
		const std::vector<Triangle>&           Triangles()       const { return m_triangles_buffer; }
		const std::vector<TriangleGroup>&      TrianglesGroups() const { return m_groups_buffer; }
		const std::map<std::string, Material>& MaterialMap()     const { return m_materials_map; }
	public:
		size_t           VertexCount()   const { return m_vertices_buffer.size(); }
		size_t           TriangleCount() const { return m_triangles_buffer.size(); }
		f32              SurfaceArea()   const { return m_surface_area; }
		const glm::vec3& Center()        const { return m_center; }
		const glm::vec3& BBoxMin()       const { return m_bboxMin; }
		const glm::vec3& BBoxMax()       const { return m_bboxMax; }
	private:
		void ComputeBounds();
		void ComputeVertexNormals();
		void ComputeTriangleNormals();
		void ComputeTriangleAreas();
		f32  ComputeSurfaceArea();
		void ComputeAll();
	private:
		void LoadOBJ();
		void LoadPLY();
	private:
		// metadata
		std::string m_filename;

		// geometry buffers
		std::vector<glm::vec3>     m_vertices_buffer;
		std::vector<glm::vec3>     m_normals_buffer;
		std::vector<glm::vec3>     m_colors_buffer;
		std::vector<glm::vec3>     m_coords_buffer;
		std::vector<Triangle>      m_triangles_buffer;
		std::vector<TriangleGroup> m_groups_buffer;

		// materials
		std::map<std::string, Material> m_materials_map;

		// spatial data
		f32       m_surface_area = 0.0f;
		glm::vec3 m_center{ 0.0f };
		glm::vec3 m_bboxMin = { F32_MAX, F32_MAX, F32_MAX };
		glm::vec3 m_bboxMax = { F32_MIN, F32_MIN, F32_MIN };
	};

}
