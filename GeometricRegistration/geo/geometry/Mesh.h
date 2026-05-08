#pragma once
#include <string>
#include <geo/utils/GeoRand.h>
#include "PointCloud3D.h"

namespace geo
{
	struct TriangleData
	{
		glm::uvec3 vertexIndices;
		glm::vec3 faceNormal;
		f32 area = 0.0f;
		const index_t& operator[](index_t i) const
		{
			return vertexIndices[i];
		}
	};

	class Mesh
	{
	public:
		Mesh() = default;
		Mesh(std::string filename, 
			std::vector<glm::vec3> points, std::vector<glm::uvec3> triangles, std::vector<glm::vec3> normals = {});
		Mesh(std::string filename, PointCloud3D pointCloud, std::vector<glm::uvec3> triangles);
		virtual ~Mesh();
	public:
		const glm::vec3& Point(index_t i) const;
		const glm::vec3& Normal(index_t i) const;
		const TriangleData& Triangle(index_t i) const;
		const glm::vec3& TriangleVertex(index_t tri, index_t corner) const;
	public:
		// geometry info
		index_t VertexCount() const { return (index_t)m_pointCloud.Size(); }
		index_t TriangleCount() const { return (index_t)m_triangles.size(); }
		const BBox& BoundingBox() const { return m_bounding_box; }
		f32 SurfaceArea() const { return m_area; }
	public:
		PointCloud3D SamplePointsUniform(u32 n, Random& rng, bool includeNormals = false) const;
	public:
		// metadata
		const std::string& FileName() const { return m_filename; }
													   
		// geometry buffers				   
		const std::vector<glm::vec3>& GetPositions() const { return m_pointCloud.GetPoints(); }
		const std::vector<glm::vec3>& GetNormals() const { return m_pointCloud.GetNormals(); }
		const std::vector<TriangleData>& GetTriangles() const { return m_triangles; }
		const PointCloud3D& GetPointCloud() const { return m_pointCloud; }
	protected:
		void ComputeVertexNormals();
		void ComputeSurfaceArea();
	protected:
		PointCloud3D m_pointCloud;
		std::vector<TriangleData> m_triangles;
	protected:
		std::string m_filename;
		BBox m_bounding_box;
		f64 m_area = 0.0;
	};

}
