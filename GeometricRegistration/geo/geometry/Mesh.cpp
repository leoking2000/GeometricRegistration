#include <iostream>
#include <fstream>
#include <sstream>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

#include "Mesh.h"


namespace geo
{
	Mesh::Mesh(std::filesystem::path filepath)
	{
		m_filename = filepath.string();

		std::filesystem::path path(filepath);
		std::string ext = path.extension().string();

		if (ext == ".obj" || ext == ".OBJ")
		{
			LoadOBJ();
		}
		else if (ext == ".ply" || ext == ".PLY")
		{
			LoadPLY();
		}
		else
		{
			throw std::runtime_error("Unsupported mesh format: " + ext);
		}

		ComputeAll();
	}

	void Mesh::LoadOBJ()
	{
		tinyobj::attrib_t attrib;
		std::vector<tinyobj::shape_t> shapes;
		std::vector<tinyobj::material_t> materials;

		std::string warn;
		std::string err;

		bool ret = tinyobj::LoadObj(
			&attrib,
			&shapes,
			&materials,
			&warn,
			&err,
			m_filename.c_str(),
			nullptr,
			true
		);

		if (!warn.empty())
			std::cout << "TinyObj warning: " << warn << std::endl;

		if (!err.empty())
			std::cerr << "TinyObj error: " << err << std::endl;

		if (!ret)
			throw std::runtime_error("Failed to load OBJ file");

		// -----------------------------
		// Vertices
		// -----------------------------
		m_vertices_buffer.reserve(attrib.vertices.size() / 3);

		for (size_t v = 0; v < attrib.vertices.size() / 3; v++)
		{
			glm::vec3 vertex;
			vertex.x = attrib.vertices[3 * v + 0];
			vertex.y = attrib.vertices[3 * v + 1];
			vertex.z = attrib.vertices[3 * v + 2];

			m_vertices_buffer.push_back(vertex);
		}

		// -----------------------------
		// Normals
		// -----------------------------
		m_normals_buffer.reserve(attrib.normals.size() / 3);

		for (size_t n = 0; n < attrib.normals.size() / 3; n++)
		{
			glm::vec3 normal;
			normal.x = attrib.normals[3 * n + 0];
			normal.y = attrib.normals[3 * n + 1];
			normal.z = attrib.normals[3 * n + 2];

			m_normals_buffer.push_back(normal);
		}

		// -----------------------------
		// Texture Coords
		// -----------------------------
		m_coords_buffer.reserve(attrib.texcoords.size() / 2);

		for (size_t t = 0; t < attrib.texcoords.size() / 2; t++)
		{
			glm::vec3 coord;
			coord.x = attrib.texcoords[2 * t + 0];
			coord.y = attrib.texcoords[2 * t + 1];
			coord.z = 0.0f;

			m_coords_buffer.push_back(coord);
		}

		// -----------------------------
		// Materials
		// -----------------------------
		for (const auto& mat : materials)
		{
			Material m;
			m.name = mat.name;

			m.baseColor = glm::vec3(
				mat.diffuse[0],
				mat.diffuse[1],
				mat.diffuse[2]
			);

			m.textureColor = mat.diffuse_texname;
			m.textureNormal = mat.normal_texname;

			m_materials_map[m.name] = m;
		}

		// -----------------------------
		// Shapes / Triangles
		// -----------------------------
		for (const auto& shape : shapes)
		{
			TriangleGroup group;
			group.start = (index_t)m_triangles_buffer.size();
			group.matname = "";

			size_t index_offset = 0;

			for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); f++)
			{
				int fv = shape.mesh.num_face_vertices[f];

				if (fv != 3)
				{
					index_offset += fv;
					continue;
				}

				Triangle tri;

				for (size_t v = 0; v < 3; v++)
				{
					const tinyobj::index_t& idx =
						shape.mesh.indices[index_offset + v];

					tri.vertex[v] = idx.vertex_index;
					tri.normal[v] = idx.normal_index;
					tri.coords[v] = idx.texcoord_index;
				}

				m_triangles_buffer.push_back(tri);

				index_offset += fv;
			}

			group.length = (index_t)m_triangles_buffer.size() - group.start;

			m_groups_buffer.push_back(group);
		}
	}

	void Mesh::LoadPLY()
	{
		std::ifstream file(m_filename);
		if (!file)
			throw std::runtime_error("Failed to open PLY file");

		std::string line;

		size_t vertexCount = 0;
		size_t faceCount = 0;

		bool header = true;

		while (header && std::getline(file, line))
		{
			std::stringstream ss(line);
			std::string token;
			ss >> token;

			if (token == "element")
			{
				ss >> token;

				if (token == "vertex")
					ss >> vertexCount;

				if (token == "face")
					ss >> faceCount;
			}

			if (token == "end_header")
				header = false;
		}

		// Load vertices
		m_vertices_buffer.reserve(vertexCount);

		for (size_t i = 0; i < vertexCount; i++)
		{
			std::getline(file, line);
			std::stringstream ss(line);

			glm::vec3 v;
			ss >> v.x >> v.y >> v.z;

			m_vertices_buffer.push_back(v);

			// optional normals
			if (!ss.eof())
			{
				glm::vec3 n;
				ss >> n.x >> n.y >> n.z;

				m_normals_buffer.push_back(n);
			}
		}

		// Load faces
		m_triangles_buffer.reserve(faceCount);

		for (size_t i = 0; i < faceCount; i++)
		{
			std::getline(file, line);
			std::stringstream ss(line);

			int verts;
			ss >> verts;

			if (verts != 3)
				continue;

			Triangle tri;

			ss >> tri.vertex[0];
			ss >> tri.vertex[1];
			ss >> tri.vertex[2];

			tri.normal[0] = tri.normal[1] = tri.normal[2] = -1;
			tri.coords[0] = tri.coords[1] = tri.coords[2] = -1;

			m_triangles_buffer.push_back(tri);
		}

		// Create default triangle group
		TriangleGroup group;
		group.start = 0;
		group.length = (index_t)m_triangles_buffer.size();
		group.matname = "default";

		m_groups_buffer.push_back(group);
	}

	void Mesh::ComputeAll()
	{
		ComputeBounds();

		if (m_normals_buffer.empty())
		{
			ComputeVertexNormals();
		}

		ComputeTriangleNormals();
		ComputeTriangleAreas();

		m_surface_area = ComputeSurfaceArea();
	}


	void Mesh::ComputeBounds()
	{
		for (const auto& v : m_vertices_buffer)
		{
			m_bboxMin = glm::min(m_bboxMin, v);
			m_bboxMax = glm::max(m_bboxMax, v);
		}

		m_center = (m_bboxMin + m_bboxMax) * 0.5f;
	}

	void Mesh::ComputeTriangleNormals()
	{
		for (auto& tri : m_triangles_buffer)
		{
			const glm::vec3& v0 = m_vertices_buffer[tri.vertex[0]];
			const glm::vec3& v1 = m_vertices_buffer[tri.vertex[1]];
			const glm::vec3& v2 = m_vertices_buffer[tri.vertex[2]];

			glm::vec3 e1 = v1 - v0;
			glm::vec3 e2 = v2 - v0;

			tri.face_normal = glm::normalize(glm::cross(e1, e2));
		}
	}

	void Mesh::ComputeVertexNormals()
	{
		m_normals_buffer.clear();
		m_normals_buffer.resize(m_vertices_buffer.size(), glm::vec3(0.0f));

		for (const auto& tri : m_triangles_buffer)
		{
			for (int i = 0; i < 3; i++)
			{
				index_t v = tri.vertex[i];
				m_normals_buffer[v] += tri.face_normal;
			}
		}

		for (auto& n : m_normals_buffer)
		{
			n = glm::normalize(n);
		}
	}

	void Mesh::ComputeTriangleAreas()
	{
		for (auto& tri : m_triangles_buffer)
		{
			const glm::vec3& v0 = m_vertices_buffer[tri.vertex[0]];
			const glm::vec3& v1 = m_vertices_buffer[tri.vertex[1]];
			const glm::vec3& v2 = m_vertices_buffer[tri.vertex[2]];

			glm::vec3 e1 = v1 - v0;
			glm::vec3 e2 = v2 - v0;

			tri.area = 0.5f * glm::length(glm::cross(e1, e2));
		}
	}

	f32 Mesh::ComputeSurfaceArea()
	{
		f32 area = 0.0f;

		for (const auto& tri : m_triangles_buffer)
		{
			area += tri.area;
		}

		return area;
	}

}