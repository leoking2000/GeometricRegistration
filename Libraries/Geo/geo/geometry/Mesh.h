#pragma once
#include <string>
#include <core/utils/Rand.h>
#include "PointCloud3D.h"

namespace geo
{
	// Per-triangle geometric data stored alongside index buffer
	struct TriangleData
	{
		glm::uvec3 vertexIndices  = glm::uvec3(0u);
		glm::vec3  faceNormal     = glm::vec3(0.0f);
		glm::vec3  centroid       = glm::vec3(0.0f);
		f32        area           = 0.0f;

		index_t operator[](int i) const
		{
			return vertexIndices[i];
		}
	};

	// Represents a triangle mesh composed of vertices, triangles, and normals.
	// Stores both raw geometry and derived geometric data (normals, bounding box, area).
	class Mesh
	{
	public:
		Mesh() = default;

		// Constructs a mesh from raw geometry buffers
		// Steps:
		// 1. Build triangle metadata (area, face normals)
		// 2. Generate vertex normals if missing
		// 3. Compute global surface area
		// 4. Compute bounding box
		// @param filename   Source filename (for metadata/debugging)
		// @param points     Vertex positions
		// @param triangles  Index buffer (triplets of vertex indices)
		// @param normals    Optional per-vertex normals (may be empty)
		Mesh(const std::string& filename, 
			std::vector<glm::vec3> points, std::vector<glm::uvec3> triangles, std::vector<glm::vec3> normals = {});
		virtual ~Mesh();
	public:
		// Returns vertex position at index i
		inline const glm::vec3& Vertex(index_t i) const { return m_vertices[i]; };
		// Returns vertex normal at index i
		inline const glm::vec3& Normal(index_t i) const { return m_normals[i]; };
		// Returns precomputed triangle data at index i
		inline const TriangleData& Triangle(index_t i) const { return m_triangles[i]; };
		// Returns position of a specific corner of a triangle
		// @param tri    Triangle index
		// @param corner Corner index (0..2)
		inline const glm::vec3& TriangleVertex(index_t tri, index_t corner) const { return m_vertices[m_triangles[tri][corner]]; };
		// Returns normal of a specific corner of a triangle
		// @param tri    Triangle index
		// @param corner Corner index (0..2)
		inline const glm::vec3& TriangleVertexNormal(index_t tri, index_t corner) const { return m_normals[m_triangles[tri][corner]]; };
	public:
		// Number of vertices in the mesh
		inline index_t VertexCount() const { return (index_t)m_vertices.size(); }
		// Number of triangles in the mesh
		inline index_t TriangleCount() const { return (index_t)m_triangles.size(); }
		// Returns axis-aligned bounding box of the mesh
		inline const core::BBox& BoundingBox() const { return m_bounding_box; }
		// Total surface area of the mesh (sum of triangle areas)
		inline f64 SurfaceArea() const { return m_area; }
	public:
		// Uniformly samples n points over the surface of the mesh
		// Optionally includes interpolated normals
		PointCloud3D SamplePointsUniform(u32 n, core::Random& rng, bool includeNormals = false) const;
		// Converts mesh vertices (and normals) into a point cloud representation
		PointCloud3D ToPointCloud() const { return PointCloud3D(m_vertices, m_normals); }
		// Flattens mesh data into a triangle soup
		void Flatten();
	public:
		// Applies a rigid transformation (rotation + translation)
		// Note:
		// - This operation mutates the internal data.
		// - Any previously built spatial acceleration structures (e.g., KD-trees, DistanceField) become invalid.
		void Transform(const core::RigidTransform& transform);
	public:
		// Source filename associated with mesh
		inline const std::string& FileName() const { return m_filename; }

		// Saves mesh to disk using geo::io system (OBJ / PLY depending on extension)
		void Save(const std::filesystem::path& path) const;

		// Loads mesh from disk (recomputes derived data internally)
		static Mesh Load(const std::filesystem::path& path);
	public:
		// Vertex positions
		inline const std::vector<glm::vec3>& GetVertices() const { return m_vertices; }
		// Vertex normals
		inline const std::vector<glm::vec3>& GetNormals() const { return m_normals; }
		// Triangle index + precomputed data
		inline const std::vector<TriangleData>& GetTriangles() const { return m_triangles; }
	protected:
		// Builds triangle metadata (normals, area) from index buffer
		void ComputeTriangleData(const std::vector<glm::uvec3>& indexBuffer);
		// Computes per-vertex normals (averaged from adjacent faces)
		void ComputeVertexNormals();
		// Computes axis-aligned bounding box of the mesh
		void ComputeBoundingBox();
		// Computes total surface area of the mesh
		void ComputeSurfaceArea();
	protected:
		std::string m_filename;                // Source file name
	protected:
		std::vector<glm::vec3> m_vertices;     // Vertex position buffer
		std::vector<glm::vec3> m_normals;      // Vertex normal buffer
		std::vector<TriangleData> m_triangles; // Triangle index buffer + cached geometric properties
	protected:
		core::BBox m_bounding_box;             // Cached bounding box of the mesh
		f64 m_area = 0.0;                      // Cached total surface area of the mesh
	};

}
