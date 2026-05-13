#include <fstream>
#include <algorithm>

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

#include <geo/utils/logging/LogMacros.h>
#include "IOUtils.h"

namespace geo::io
{
    // ============================================================
    // GeometryDumpData
    // ============================================================

    Mesh GeometryDumpData::ToMesh() const
    {
        return Mesh(filePath.filename().string(), points, indexBuffer, normals);
    }

    PointCloud3D GeometryDumpData::ToPointCloud() const
    {
        return PointCloud3D(points, normals);
    }

    // ============================================================
    // Utilities
    // ============================================================

    bool FileExists(const std::filesystem::path& path)
    {
        return std::filesystem::exists(path);
    }

    FileType GetFileType(const std::filesystem::path& path)
    {
        if (!path.has_extension())
        {
            return FileType::UNKNOWN;
        }

        std::string ext = path.extension().string();

        std::transform(ext.begin(), ext.end(), ext.begin(),
            [](unsigned char c)
            {
                return static_cast<char>(std::tolower(c));
            });

        if (ext == ".obj")
        {
            return FileType::OBJ;
        }

        if (ext == ".ply")
        {
            return FileType::PLY;
        }

        return FileType::UNKNOWN;
    }

    std::string GetFileName(const std::filesystem::path& path)
    {
        return path.filename().string();
    }

    // ============================================================
    // OBJ Loader
    // ============================================================

    GeometryDumpData LoadOBJ(const std::filesystem::path& path)
    {
        GeometryDumpData data;

        data.filePath = path;
        data.fileType = FileType::OBJ;
        data.geometryType = GeometryType::TRIANGLE_MESH;

        tinyobj::attrib_t attrib;
        std::vector<tinyobj::shape_t> shapes;
        std::vector<tinyobj::material_t> materials;

        std::string warn;
        std::string err;

        bool success = tinyobj::LoadObj(
            &attrib,
            &shapes,
            &materials,
            &warn,
            &err,
            path.string().c_str(),
            nullptr,
            true
        );

        if (!warn.empty())
        {
            GEOLOGWARN(warn);
        }

        if (!err.empty())
        {
            GEOLOGERROR(err);
        }

        if (!success)
        {
            GEOLOGERROR("Failed to load OBJ: " << path);
            return {};
        }

        // --------------------------------------------------------
        // Vertices
        // --------------------------------------------------------

        data.points.reserve(attrib.vertices.size() / 3);

        for (size_t i = 0; i < attrib.vertices.size(); i += 3)
        {
            data.points.emplace_back(
                attrib.vertices[i + 0],
                attrib.vertices[i + 1],
                attrib.vertices[i + 2]
            );
        }

        // --------------------------------------------------------
        // Normals
        // --------------------------------------------------------

        if (!attrib.normals.empty())
        {
            data.normals.reserve(attrib.normals.size() / 3);

            for (size_t i = 0; i < attrib.normals.size(); i += 3)
            {
                data.normals.emplace_back(
                    attrib.normals[i + 0],
                    attrib.normals[i + 1],
                    attrib.normals[i + 2]
                );
            }
        }

        // --------------------------------------------------------
        // Triangles
        // --------------------------------------------------------

        for (const auto& shape : shapes)
        {
            size_t indexOffset = 0;

            for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); f++)
            {
                int fv = shape.mesh.num_face_vertices[f];

                if (fv != 3)
                {
                    GEOLOGWARN("Skipping non-triangle face in OBJ file");
                    indexOffset += fv;
                    continue;
                }

                glm::uvec3 tri(0);

                for (int v = 0; v < 3; v++)
                {
                    const tinyobj::index_t idx = shape.mesh.indices[indexOffset + v];
                    tri[v] = (u32)idx.vertex_index;
                }

                data.indexBuffer.emplace_back(tri);

                indexOffset += 3;
            }
        }

        return data;
    }

    // ============================================================
    // Geometry Loading
    // ============================================================

    GeometryDumpData LoadGeometry(const std::filesystem::path& path)
    {
        if (!FileExists(path))
        {
            GEOLOGERROR("Geometry file does not exist: " << path);
            return {};
        }

        const FileType type = GetFileType(path);

        switch (type)
        {
            case FileType::OBJ:
            {
                return LoadOBJ(path);
            }
            case FileType::PLY:
            {
                GEOLOGERROR("PLY loading not implemented yet");
                return {};
            }
            default:
            {
                GEOLOGERROR("Unsupported geometry file type: " << path);
                return {};
            }
        }
    }

    // ============================================================
    // Geometry Saving
    // ============================================================

    bool SaveOBJ(const std::filesystem::path& path, const GeometryDumpData& data)
    {
        std::ofstream file(path);

        if (!file.is_open())
        {
            GEOLOGERROR("Failed to open OBJ for writing: " << path);
            return false;
        }

        // --------------------------------------------------------
        // vertices
        // --------------------------------------------------------

        for (const glm::vec3& p : data.points)
        {
            file << "v " << p.x << " " << p.y << " " << p.z << "\n";
        }

        // --------------------------------------------------------
        // normals
        // --------------------------------------------------------

        for (const glm::vec3& n : data.normals)
        {
            file << "vn " << n.x << " " << n.y << " " << n.z << "\n";
        }

        // --------------------------------------------------------
        // triangles
        // --------------------------------------------------------

        for (const glm::uvec3& tri : data.indexBuffer)
        {
            file << "f ";

            for (int i = 0; i < 3; i++)
            {
                const u32 idx = tri[i] + 1;

                if (data.HasNormals())
                {
                    file << idx << "//" << idx;
                }
                else
                {
                    file << idx;
                }

                if (i < 2)
                {
                    file << " ";
                }
            }

            file << "\n";
        }

        return true;
    }

    void SaveGeometry(const std::filesystem::path& path, const GeometryDumpData& data)
    {
        const FileType type = GetFileType(path);

        switch (type)
        {
            case FileType::OBJ:
            {
                SaveOBJ(path, data);
                return;
            }
            case FileType::PLY:
            {
                GEOLOGERROR("PLY saving not implemented yet");
                return;
            }
            default:
            {
                GEOLOGERROR("Unsupported geometry file type");
                return;
            }
        }
    }

}
