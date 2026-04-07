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
		m_filename = filepath.filename().string();
		std::string ext = filepath.extension().string();

		if (ext == ".obj" || ext == ".OBJ")
		{
			LoadOBJ(filepath);
		}
		else if (ext == ".ply" || ext == ".PLY")
		{
			//LoadPLY(filepath);
			throw std::runtime_error("Unsupported mesh format: " + ext);
		}
		else
		{
			throw std::runtime_error("Unsupported mesh format: " + ext);
		}

		if (m_vertices_buffer.empty())
		{
			throw std::runtime_error("Mesh contains no vertices: " + m_filename);
		}

		if (m_triangles_buffer.empty())
		{
			throw std::runtime_error("Mesh contains no triangles: " + m_filename);
		}

		ComputeAll();
	}

	void Mesh::LoadOBJ(const std::filesystem::path& filepath)
	{
        tinyobj::ObjReaderConfig config;
        config.mtl_search_path = filepath.parent_path().string();

        tinyobj::ObjReader reader;
        if (!reader.ParseFromFile(filepath.string(), config))
        {
            std::string err = reader.Error();
            if (err.empty())
            {
                err = "Failed to load OBJ: " + filepath.string();
            }
            throw std::runtime_error(err);
        }

        if (!reader.Warning().empty())
        {
            // Optional: log warnings if you want
        }

        const tinyobj::attrib_t& attrib = reader.GetAttrib();
        const std::vector<tinyobj::shape_t>& shapes = reader.GetShapes();
        const std::vector<tinyobj::material_t>& materials = reader.GetMaterials();

        // Vertices
        m_vertices_buffer.reserve(attrib.vertices.size() / 3);
        for (size_t i = 0; i + 2 < attrib.vertices.size(); i += 3)
        {
            m_vertices_buffer.emplace_back(
                attrib.vertices[i + 0],
                attrib.vertices[i + 1],
                attrib.vertices[i + 2]);
        }

        // Normals
        if (!attrib.normals.empty())
        {
            m_normals_buffer.reserve(attrib.normals.size() / 3);
            for (size_t i = 0; i + 2 < attrib.normals.size(); i += 3)
            {
                m_normals_buffer.emplace_back(
                    attrib.normals[i + 0],
                    attrib.normals[i + 1],
                    attrib.normals[i + 2]);
            }
        }

        // Colors if present
        if (!attrib.colors.empty())
        {
            m_colors_buffer.reserve(attrib.colors.size() / 3);
            for (size_t i = 0; i + 2 < attrib.colors.size(); i += 3)
            {
                m_colors_buffer.emplace_back(
                    attrib.colors[i + 0],
                    attrib.colors[i + 1],
                    attrib.colors[i + 2]);
            }
        }

        // Texcoords as vec3, z = 0
        if (!attrib.texcoords.empty())
        {
            m_coords_buffer.reserve(attrib.texcoords.size() / 2);
            for (size_t i = 0; i + 1 < attrib.texcoords.size(); i += 2)
            {
                m_coords_buffer.emplace_back(
                    attrib.texcoords[i + 0],
                    attrib.texcoords[i + 1],
                    0.0f);
            }
        }

        // Materials
        for (const auto& mtl : materials)
        {
            Material m;
            m.name = mtl.name.empty() ? "default" : mtl.name;
            m.baseColor = glm::vec3(mtl.diffuse[0], mtl.diffuse[1], mtl.diffuse[2]);

            if (!mtl.diffuse_texname.empty())
                m.textureColor = mtl.diffuse_texname;

            if (!mtl.bump_texname.empty())
                m.textureNormal = mtl.bump_texname;

            if (!mtl.alpha_texname.empty())
                m.textureMask = mtl.alpha_texname;

            m_materials_map[m.name] = m;
        }

        // Triangles + groups
        for (const auto& shape : shapes)
        {
            size_t indexOffset = 0;
            index_t groupStart = static_cast<index_t>(m_triangles_buffer.size());

            // tinyobj material ids are per-face
            std::string groupMaterialName = "default";

            for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); ++f)
            {
                const int fv = shape.mesh.num_face_vertices[f];
                if (fv < 3)
                {
                    indexOffset += static_cast<size_t>(fv);
                    continue;
                }

                const int materialId =
                    (f < shape.mesh.material_ids.size()) ? shape.mesh.material_ids[f] : -1;

                if (materialId >= 0 && static_cast<size_t>(materialId) < materials.size())
                {
                    groupMaterialName = materials[materialId].name.empty()
                        ? "default"
                        : materials[materialId].name;
                }

                // Fan triangulation
                const tinyobj::index_t i0 = shape.mesh.indices[indexOffset + 0];

                for (int k = 1; k + 1 < fv; ++k)
                {
                    const tinyobj::index_t i1 = shape.mesh.indices[indexOffset + k];
                    const tinyobj::index_t i2 = shape.mesh.indices[indexOffset + k + 1];

                    Triangle tri{};

                    tri.vertex[0] = static_cast<index_t>(i0.vertex_index);
                    tri.vertex[1] = static_cast<index_t>(i1.vertex_index);
                    tri.vertex[2] = static_cast<index_t>(i2.vertex_index);

                    tri.normal[0] = (i0.normal_index >= 0) ? static_cast<index_t>(i0.normal_index) : 0;
                    tri.normal[1] = (i1.normal_index >= 0) ? static_cast<index_t>(i1.normal_index) : 0;
                    tri.normal[2] = (i2.normal_index >= 0) ? static_cast<index_t>(i2.normal_index) : 0;

                    tri.coords[0] = (i0.texcoord_index >= 0) ? static_cast<index_t>(i0.texcoord_index) : 0;
                    tri.coords[1] = (i1.texcoord_index >= 0) ? static_cast<index_t>(i1.texcoord_index) : 0;
                    tri.coords[2] = (i2.texcoord_index >= 0) ? static_cast<index_t>(i2.texcoord_index) : 0;

                    m_triangles_buffer.push_back(tri);
                }

                indexOffset += static_cast<size_t>(fv);
            }

            const index_t groupEnd = static_cast<index_t>(m_triangles_buffer.size());
            if (groupEnd > groupStart)
            {
                TriangleGroup group;
                group.start = groupStart;
                group.length = groupEnd - groupStart;
                group.matname = groupMaterialName;
                m_groups_buffer.push_back(group);
            }
        }

        if (m_materials_map.empty())
        {
            m_materials_map["default"] = Material{};
        }

        // If OBJ normals exist but are per-corner rather than one-per-vertex,
        // do not pretend they are vertex normals unless counts match.
        if (!m_normals_buffer.empty() && m_normals_buffer.size() != m_vertices_buffer.size())
        {
            m_normals_buffer.clear();
        }
	}

	//void Mesh::LoadPLY(std::filesystem::path filepath)
	//{
	//
	//}

	void Mesh::ComputeAll()
	{
		ComputeBounds();
		ComputeTriangleNormals();
		ComputeTriangleAreas();

		if (m_normals_buffer.empty())
		{
			ComputeVertexNormals();
		}

		m_surface_area = ComputeSurfaceArea();
	}


	void Mesh::ComputeBounds()
	{
		assert(!m_vertices_buffer.empty());

		m_bboxMin = glm::vec3(F32_MAX, F32_MAX, F32_MAX);
		m_bboxMax = glm::vec3(-F32_MAX, -F32_MAX, -F32_MAX);

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

			const glm::vec3 e1 = v1 - v0;
			const glm::vec3 e2 = v2 - v0;

			const glm::vec3 c = glm::cross(e1, e2);
			const f32 len2 = glm::dot(c, c);

			if (len2 > 0.0f)
			{
				tri.face_normal = c / std::sqrt(len2);
			}
			else
			{
				tri.face_normal = glm::vec3(0.0f);
			}
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
				const index_t v = tri.vertex[i];
				assert(v < m_normals_buffer.size());
				m_normals_buffer[v] += tri.face_normal;
			}
		}

		for (auto& n : m_normals_buffer)
		{
			const f32 len2 = glm::dot(n, n);
			if (len2 > 0.0f)
			{
				n /= std::sqrt(len2);
			}
			else
			{
				n = glm::vec3(0.0f, 1.0f, 0.0f);
			}
		}
	}

	void Mesh::ComputeTriangleAreas()
	{
		for (auto& tri : m_triangles_buffer)
		{
			const glm::vec3& v0 = m_vertices_buffer[tri.vertex[0]];
			const glm::vec3& v1 = m_vertices_buffer[tri.vertex[1]];
			const glm::vec3& v2 = m_vertices_buffer[tri.vertex[2]];

			const glm::vec3 e1 = v1 - v0;
			const glm::vec3 e2 = v2 - v0;

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