#pragma once
#include <vector>
#include "PointCloud3D.h"

namespace geo
{
	class Mesh
	{
	public:
		explicit Mesh(const std::string& filename, 
			std::vector<glm::vec3> vertex_buffer, 
			std::vector<glm::ivec3> index_buffer, 
			std::vector<glm::vec3> normal_buffer,
			std::vector<glm::vec2> coords_buffer
		);

		virtual ~Mesh();
	public:
		const std::string&                     FileName()        const { return m_filename; }

		const std::vector<glm::vec3>&          Vertices()        const { return m_vertices_buffer; }
		const std::vector<glm::ivec3>&		   Indexes()         const { return m_index_buffer; }
		const std::vector<glm::vec3>&          Normals()         const { return m_normals_buffer; }
		const std::vector<glm::vec2>&          Coords()          const { return m_coords_buffer; }
	public:
		size_t           VertexCount()   const { return m_vertices_buffer.size(); }
		size_t           TriangleCount() const { return m_index_buffer.size(); }
		const BBox&      BoundingBox()   const { return m_boundingBox; };
		PointCloud3D     ToPointCloud()  const { return PointCloud3D(m_vertices_buffer, m_normals_buffer); }
	public:
		void flatten();
	protected:
		void ComputeBoundingBox();
	protected:
		// metadata
		std::string m_filename;

		// geometry buffers
		std::vector<glm::vec3>     m_vertices_buffer;
		std::vector<glm::ivec3>    m_index_buffer;
		std::vector<glm::vec3>     m_normals_buffer;
		std::vector<glm::vec2>     m_coords_buffer;

		// spatial data
		BBox m_boundingBox;
	};

}
