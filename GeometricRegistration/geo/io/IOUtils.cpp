#include <fstream>
#include <sstream>
#include <algorithm>
#include <cstring>

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

    std::string ReadFile(const std::string& filepath)
    {
        std::ifstream input_file(filepath, std::ios::binary);

        if (!input_file.is_open())
        {
            GEOLOGERROR("Failed to open file at " << filepath.c_str());
            return std::string();
        }

        return std::string((std::istreambuf_iterator<char>(input_file)), std::istreambuf_iterator<char>());
    }

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

    std::filesystem::path GetParentFolder(const std::filesystem::path& filePath)
    {
        return filePath.parent_path();
    }

    // ============================================================
    // OBJ Loader
    // ============================================================

    struct VertexPN
    {
        glm::vec3 pos;
        glm::vec3 normal;
    };

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
            &attrib, &shapes, &materials,
            &warn, &err,
            path.string().c_str(),
            path.parent_path().string().c_str(),
            true // triangulate
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
        //
        // NOTE: OBJ stores per-face-vertex normal indices which can differ
        // from vertex indices (e.g. "f 1//3 2//1 3//2"). We load the raw
        // normal buffer here and store only vertex_index in the index buffer.
        // If normal count doesn't match vertex count, Mesh will recompute
        // normals via ComputeVertexNormals().
        // --------------------------------------------------------

        //if (!attrib.normals.empty())
        //{
        //    data.normals.reserve(attrib.normals.size() / 3);
        //    for (size_t i = 0; i < attrib.normals.size(); i += 3)
        //    {
        //        data.normals.emplace_back(
        //            attrib.normals[i + 0],
        //            attrib.normals[i + 1],
        //            attrib.normals[i + 2]
        //        );
        //    }
        //}

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

        GEOLOGINFO("Loaded OBJ: " << path
            << " | " << data.points.size() << " vertices"
            << " | " << data.indexBuffer.size() << " triangles"
            << " | normals: " << (data.HasNormals() ? "yes" : "no"));

        return data;
    }

    // ============================================================
    // PLY Loader
    //
    // Supports:
    //   - ASCII, binary_little_endian, binary_big_endian formats
    //   - Vertex element with x/y/z positions and nx/ny/nz normals (optional)
    //   - Face element with vertex_indices or vertex_index list property
    //   - Triangle faces and quad faces (quads are split into two triangles)
    //   - Unknown properties are skipped safely
    //   - Auto-detects TRIANGLE_MESH vs POINT_CLOUD based on face presence
    // ============================================================

    namespace PLY
    {
        // PLY on-disk format
        enum class PLYFormat { ASCII, BINARY_LE, BINARY_BE, UNKNOWN };

        // Scalar types used in PLY property declarations
        enum class PLYType { F32, F64, I8, U8, I16, U16, I32, U32, UNKNOWN };

        static inline PLYType StringToPLYType(const std::string& s)
        {
            if (s == "float" || s == "float32") return PLYType::F32;
            if (s == "double" || s == "float64") return PLYType::F64;
            if (s == "char" || s == "int8")    return PLYType::I8;
            if (s == "uchar" || s == "uint8")   return PLYType::U8;
            if (s == "short" || s == "int16")   return PLYType::I16;
            if (s == "ushort" || s == "uint16")  return PLYType::U16;
            if (s == "int" || s == "int32")   return PLYType::I32;
            if (s == "uint" || s == "uint32")  return PLYType::U32;
            return PLYType::UNKNOWN;
        }

        static inline size_t PLYTypeBytes(PLYType t)
        {
            switch (t)
            {
            case PLYType::F32: return 4;
            case PLYType::F64: return 8;
            case PLYType::I8:  return 1;
            case PLYType::U8:  return 1;
            case PLYType::I16: return 2;
            case PLYType::U16: return 2;
            case PLYType::I32: return 4;
            case PLYType::U32: return 4;
            default:           return 0;
            }
        }

        struct PLYProperty
        {
            std::string name;
            PLYType     type = PLYType::UNKNOWN; // scalar type (if not list)
            bool        isList = false;
            PLYType     listCount = PLYType::UNKNOWN; // type of count field
            PLYType     listElem = PLYType::UNKNOWN; // type of each list element
        };

        struct PLYElement
        {
            std::string              name;
            size_t                   count = 0;
            std::vector<PLYProperty> props;

            // Returns the index of a named property, or -1 if not found
            int FindProp(const std::string& n) const
            {
                for (int i = 0; i < (int)props.size(); i++)
                    if (props[i].name == n) return i;
                return -1;
            }
        };

        // --------------------------------------------------------
        // Binary read helpers
        //
        // SwapBytes=true for big-endian files.
        // All reads use memcpy to avoid strict-aliasing UB.
        // ptr is advanced by the size of the type read.
        // --------------------------------------------------------

        static inline u16 ByteSwap16(u16 v)
        {
            return (v >> 8) | (v << 8);
        }

        static inline u32 ByteSwap32(u32 v)
        {
            return ((v & 0x000000FFu) << 24) |
                ((v & 0x0000FF00u) << 8) |
                ((v & 0x00FF0000u) >> 8) |
                ((v & 0xFF000000u) >> 24);
        }

        static inline u64 ByteSwap64(u64 v)
        {
            v = ((v & 0x00000000FFFFFFFFull) << 32) | ((v >> 32) & 0x00000000FFFFFFFFull);
            v = ((v & 0x0000FFFF0000FFFFull) << 16) | ((v >> 16) & 0x0000FFFF0000FFFFull);
            v = ((v & 0x00FF00FF00FF00FFull) << 8) | ((v >> 8) & 0x00FF00FF00FF00FFull);
            return v;
        }

        // Read a PLY scalar value from ptr as float, advancing ptr
        template<bool SwapBytes>
        float BinaryReadFloat(const u8*& ptr, PLYType t)
        {
            switch (t)
            {
            case PLYType::F32:
            {
                u32 raw; std::memcpy(&raw, ptr, 4); ptr += 4;
                if constexpr (SwapBytes) raw = ByteSwap32(raw);
                float v; std::memcpy(&v, &raw, 4);
                return v;
            }
            case PLYType::F64:
            {
                u64 raw; std::memcpy(&raw, ptr, 8); ptr += 8;
                if constexpr (SwapBytes) raw = ByteSwap64(raw);
                double v; std::memcpy(&v, &raw, 8);
                return static_cast<float>(v);
            }
            case PLYType::I8: { i8  v = *reinterpret_cast<const i8*>(ptr);  ptr += 1; return static_cast<float>(v); }
            case PLYType::U8: { u8  v = *ptr;                                ptr += 1; return static_cast<float>(v); }
            case PLYType::I16:
            {
                u16 raw; std::memcpy(&raw, ptr, 2); ptr += 2;
                if constexpr (SwapBytes) raw = ByteSwap16(raw);
                i16 v; std::memcpy(&v, &raw, 2);
                return static_cast<float>(v);
            }
            case PLYType::U16:
            {
                u16 raw; std::memcpy(&raw, ptr, 2); ptr += 2;
                if constexpr (SwapBytes) raw = ByteSwap16(raw);
                return static_cast<float>(raw);
            }
            case PLYType::I32:
            {
                u32 raw; std::memcpy(&raw, ptr, 4); ptr += 4;
                if constexpr (SwapBytes) raw = ByteSwap32(raw);
                i32 v; std::memcpy(&v, &raw, 4);
                return static_cast<float>(v);
            }
            case PLYType::U32:
            {
                u32 raw; std::memcpy(&raw, ptr, 4); ptr += 4;
                if constexpr (SwapBytes) raw = ByteSwap32(raw);
                return static_cast<float>(raw);
            }
            default: return 0.0f;
            }
        }

        // Read a PLY scalar value from ptr as u32, advancing ptr
        template<bool SwapBytes>
        u32 BinaryReadUInt(const u8*& ptr, PLYType t)
        {
            return static_cast<u32>(BinaryReadFloat<SwapBytes>(ptr, t));
        }

        // Skip over a single property in the binary stream, advancing ptr
        template<bool SwapBytes>
        void BinarySkipProp(const u8*& ptr, const PLYProperty& prop)
        {
            if (prop.isList)
            {
                u32 count = BinaryReadUInt<SwapBytes>(ptr, prop.listCount);
                ptr += count * PLYTypeBytes(prop.listElem);
            }
            else
            {
                ptr += PLYTypeBytes(prop.type);
            }
        }

    } // PLY namespace

    GeometryDumpData LoadPLY(const std::filesystem::path& path)
    {
        GeometryDumpData data;
        data.filePath = path;
        data.fileType = FileType::PLY;

        // Open in binary mode — required for binary PLY; works for ASCII too
        // because we parse the header line by line and then handle data separately
        std::ifstream file(path, std::ios::binary);
        if (!file.is_open())
        {
            GEOLOGERROR("Failed to open PLY file: " << path);
            return {};
        }

        // --------------------------------------------------------
        // 1. Parse header
        // --------------------------------------------------------

        PLY::PLYFormat format = PLY::PLYFormat::UNKNOWN;
        std::vector<PLY::PLYElement> elements;

        {
            std::string line;

            // First line must be "ply"
            std::getline(file, line);
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (line != "ply")
            {
                GEOLOGERROR("Not a valid PLY file (missing magic): " << path);
                return {};
            }

            while (std::getline(file, line))
            {
                // Strip Windows carriage return
                if (!line.empty() && line.back() == '\r') line.pop_back();

                std::istringstream ss(line);
                std::string token;
                ss >> token;

                if (token == "end_header")
                {
                    break; // data begins at current file position
                }
                else if (token == "format")
                {
                    std::string fmt;
                    ss >> fmt;
                    if (fmt == "ascii")                       format = PLY::PLYFormat::ASCII;
                    else if (fmt == "binary_little_endian")   format = PLY::PLYFormat::BINARY_LE;
                    else if (fmt == "binary_big_endian")      format = PLY::PLYFormat::BINARY_BE;
                }
                else if (token == "element")
                {
                    PLY::PLYElement elem;
                    ss >> elem.name >> elem.count;
                    elements.push_back(std::move(elem));
                }
                else if (token == "property")
                {
                    if (elements.empty()) continue;

                    PLY::PLYProperty prop;
                    std::string typeStr;
                    ss >> typeStr;

                    if (typeStr == "list")
                    {
                        // "property list <count_type> <elem_type> <name>"
                        prop.isList = true;
                        std::string countTypeStr, elemTypeStr;
                        ss >> countTypeStr >> elemTypeStr >> prop.name;
                        prop.listCount = PLY::StringToPLYType(countTypeStr);
                        prop.listElem = PLY::StringToPLYType(elemTypeStr);
                    }
                    else
                    {
                        // "property <type> <name>"
                        prop.type = PLY::StringToPLYType(typeStr);
                        ss >> prop.name;
                    }

                    elements.back().props.push_back(std::move(prop));
                }
                // "comment", "obj_info" etc. are intentionally ignored
            }
        }

        if (format == PLY::PLYFormat::UNKNOWN)
        {
            GEOLOGERROR("PLY file has unknown or missing format declaration: " << path);
            return {};
        }

        // --------------------------------------------------------
        // 2. Locate vertex and face elements
        // --------------------------------------------------------

        const PLY::PLYElement* vertElem = nullptr;
        const PLY::PLYElement* faceElem = nullptr;

        for (const auto& e : elements)
        {
            if (e.name == "vertex") vertElem = &e;
            if (e.name == "face")   faceElem = &e;
        }

        if (!vertElem || vertElem->count == 0)
        {
            GEOLOGERROR("PLY file has no vertex element: " << path);
            return {};
        }

        // Find property indices within the vertex element
        const int xIdx  = vertElem->FindProp("x");
        const int yIdx  = vertElem->FindProp("y");
        const int zIdx  = vertElem->FindProp("z");
        const int nxIdx = vertElem->FindProp("nx");
        const int nyIdx = vertElem->FindProp("ny");
        const int nzIdx = vertElem->FindProp("nz");

        if (xIdx < 0 || yIdx < 0 || zIdx < 0)
        {
            GEOLOGERROR("PLY vertex element is missing x/y/z properties: " << path);
            return {};
        }

        // normals are optional
        const bool hasNormals = (nxIdx >= 0 && nyIdx >= 0 && nzIdx >= 0);

        // face list property — try both common names
        int faceListPropIdx = -1;
        if (faceElem)
        {
            faceListPropIdx = faceElem->FindProp("vertex_indices");
            if (faceListPropIdx < 0)
                faceListPropIdx = faceElem->FindProp("vertex_index");
        }

        data.points.reserve(vertElem->count);
        if (hasNormals)
            data.normals.reserve(vertElem->count);
        if (faceElem)
            data.indexBuffer.reserve(faceElem->count);

        // --------------------------------------------------------
        // 3. Read data
        // --------------------------------------------------------

        if (format == PLY::PLYFormat::ASCII)
        {
            // --- ASCII vertex read ---
            // Each line contains all property values for one vertex, in declaration order.
            // We tokenize the line and index directly into the token array.

            std::string line;
            for (size_t i = 0; i < vertElem->count; i++)
            {
                if (!std::getline(file, line)) break;
                if (!line.empty() && line.back() == '\r') line.pop_back();

                std::istringstream ss(line);
                std::vector<std::string> tokens;
                {
                    std::string t;
                    while (ss >> t) tokens.push_back(std::move(t));
                }

                const int maxScalarIdx = std::max({ xIdx, yIdx, zIdx });
                if ((int)tokens.size() <= maxScalarIdx) continue;

                data.points.emplace_back(
                    std::stof(tokens[xIdx]),
                    std::stof(tokens[yIdx]),
                    std::stof(tokens[zIdx])
                );

                if (hasNormals)
                {
                    const int maxNIdx = std::max({ nxIdx, nyIdx, nzIdx });
                    if ((int)tokens.size() > maxNIdx)
                    {
                        data.normals.emplace_back(
                            std::stof(tokens[nxIdx]),
                            std::stof(tokens[nyIdx]),
                            std::stof(tokens[nzIdx])
                        );
                    }
                }
            }

            // --- ASCII face read ---
            if (faceElem && faceListPropIdx >= 0)
            {
                for (size_t i = 0; i < faceElem->count; i++)
                {
                    if (!std::getline(file, line)) break;
                    if (!line.empty() && line.back() == '\r') line.pop_back();

                    std::istringstream ss(line);
                    int count;
                    ss >> count;

                    if (count == 3)
                    {
                        u32 a, b, c;
                        ss >> a >> b >> c;
                        data.indexBuffer.emplace_back(a, b, c);
                    }
                    else if (count == 4)
                    {
                        // Fan triangulation of quads: (a,b,c) and (a,c,d)
                        u32 a, b, c, d;
                        ss >> a >> b >> c >> d;
                        data.indexBuffer.emplace_back(a, b, c);
                        data.indexBuffer.emplace_back(a, c, d);
                    }
                    else
                    {
                        GEOLOGWARN("PLY: skipping polygon with " << count << " vertices");
                    }
                }
            }
        }
        else
        {
            // --- Binary read (little-endian or big-endian) ---
            //
            // Read remaining file bytes into memory. This avoids per-property
            // seeks and lets us advance a raw pointer — much faster than
            // repeated istream reads for large scan files.

            std::vector<u8> raw(
                (std::istreambuf_iterator<char>(file)),
                std::istreambuf_iterator<char>()
            );
            const u8* ptr = raw.data();
            const u8* end = raw.data() + raw.size();

            const bool isBigEndian = (format == PLY::PLYFormat::BINARY_BE);

            // Lambdas dispatch to the correct template instantiation at load time,
            // avoiding a branch inside the inner loop over millions of vertices.
            auto readFloat = [&](PLY::PLYType t) -> float
                {
                    return isBigEndian
                        ? PLY::BinaryReadFloat<true>(ptr, t)
                        : PLY::BinaryReadFloat<false>(ptr, t);
                };

            auto readUInt = [&](PLY::PLYType t) -> u32
                {
                    return isBigEndian
                        ? PLY::BinaryReadUInt<true>(ptr, t)
                        : PLY::BinaryReadUInt<false>(ptr, t);
                };

            auto skipProp = [&](const PLY::PLYProperty& prop)
                {
                    isBigEndian
                        ? PLY::BinarySkipProp<true>(ptr, prop)
                        : PLY::BinarySkipProp<false>(ptr, prop);
                };

            // --- Binary vertex read ---
            for (size_t i = 0; i < vertElem->count; i++)
            {
                if (ptr >= end) break;

                float x = 0, y = 0, z = 0;
                float nx = 0, ny = 0, nz = 0;

                for (int p = 0; p < (int)vertElem->props.size(); p++)
                {
                    const PLY::PLYProperty& prop = vertElem->props[p];

                    // Read known scalar properties, skip everything else.
                    // isList vertex properties are unusual but skipped safely.
                    if (prop.isList)
                    {
                        skipProp(prop);
                    }
                    else if (p == xIdx)  x  = readFloat(prop.type);
                    else if (p == yIdx)  y  = readFloat(prop.type);
                    else if (p == zIdx)  z  = readFloat(prop.type);
                    else if (p == nxIdx) nx = readFloat(prop.type);
                    else if (p == nyIdx) ny = readFloat(prop.type);
                    else if (p == nzIdx) nz = readFloat(prop.type);
                    else                 ptr += PLY::PLYTypeBytes(prop.type); // skip unknown scalar
                }

                data.points.emplace_back(x, y, z);
                if (hasNormals)
                    data.normals.emplace_back(nx, ny, nz);
            }

            // --- Binary face read ---
            if (faceElem && faceListPropIdx >= 0)
            {
                for (size_t i = 0; i < faceElem->count; i++)
                {
                    if (ptr >= end) break;

                    for (int p = 0; p < (int)faceElem->props.size(); p++)
                    {
                        const PLY::PLYProperty& prop = faceElem->props[p];

                        if (p == faceListPropIdx && prop.isList)
                        {
                            u32 count = readUInt(prop.listCount);

                            if (count == 3)
                            {
                                u32 a = readUInt(prop.listElem);
                                u32 b = readUInt(prop.listElem);
                                u32 c = readUInt(prop.listElem);
                                data.indexBuffer.emplace_back(a, b, c);
                            }
                            else if (count == 4)
                            {
                                u32 a = readUInt(prop.listElem);
                                u32 b = readUInt(prop.listElem);
                                u32 c = readUInt(prop.listElem);
                                u32 d = readUInt(prop.listElem);
                                data.indexBuffer.emplace_back(a, b, c);
                                data.indexBuffer.emplace_back(a, c, d);
                            }
                            else
                            {
                                // Skip n-gon (n>4): advance past all index bytes
                                GEOLOGWARN("PLY: skipping polygon with " << count << " vertices");
                                ptr += count * PLY::PLYTypeBytes(prop.listElem);
                            }
                        }
                        else
                        {
                            skipProp(prop);
                        }
                    }
                }
            }
        }
        // --------------------------------------------------------
        // 4. Determine geometry type
        // --------------------------------------------------------

        data.geometryType = data.HasIndices()
            ? GeometryType::TRIANGLE_MESH
            : GeometryType::POINT_CLOUD;

        GEOLOGINFO("Loaded PLY: " << path
            << " | " << data.points.size() << " vertices"
            << " | " << data.indexBuffer.size() << " triangles"
            << " | normals: " << (hasNormals ? "yes" : "no"));

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
                return LoadPLY(path);
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

    bool SavePLY(const std::filesystem::path& path, const GeometryDumpData& data)
    {
        if (data.points.empty())
        {
            GEOLOGERROR("SavePLY: no points to write");
            return false;
        }

        // Binary mode is required — text mode on Windows would corrupt binary data
        // by translating \n to \r\n inside the data section
        std::ofstream file(path, std::ios::binary);
        if (!file.is_open())
        {
            GEOLOGERROR("SavePLY: failed to open file for writing: " << path);
            return false;
        }

        const bool hasNormals = data.HasNormals();
        const bool hasFaces = data.HasIndices();

        // --------------------------------------------------------
        // 1. Write ASCII header
        //
        // The header is always ASCII text even in binary PLY files.
        // It must exactly match the binary layout that follows —
        // declared properties, declared order, declared types.
        // --------------------------------------------------------

        file << "ply\n";
        file << "format binary_little_endian 1.0\n";
        file << "comment Created by geo\n";
        file << "element vertex " << data.points.size() << "\n";
        file << "property float x\n";
        file << "property float y\n";
        file << "property float z\n";

        if (hasNormals)
        {
            file << "property float nx\n";
            file << "property float ny\n";
            file << "property float nz\n";
        }

        if (hasFaces)
        {
            file << "element face " << data.indexBuffer.size() << "\n";
            // uchar count (always 3 for triangles) + uint per index
            file << "property list uchar uint vertex_indices\n";
        }

        file << "end_header\n";

        // --------------------------------------------------------
        // 2. Pack vertex data into a contiguous buffer
        //
        // One large write is far faster than per-vertex or per-float writes.
        // We use memcpy to move float bytes — avoids strict-aliasing UB.
        //
        // NOTE: This writes floats in native byte order. We declare
        // binary_little_endian in the header, which is correct on x86/x64.
        // On a big-endian host you would need to byte-swap each value.
        // --------------------------------------------------------

        {
            const size_t floatsPerVertex = hasNormals ? 6 : 3;
            const size_t vertexBytes = data.points.size() * floatsPerVertex * sizeof(float);

            std::vector<u8> buf(vertexBytes);
            u8* ptr = buf.data();

            for (size_t i = 0; i < data.points.size(); i++)
            {
                const glm::vec3& p = data.points[i];
                std::memcpy(ptr, &p.x, 4); ptr += 4;
                std::memcpy(ptr, &p.y, 4); ptr += 4;
                std::memcpy(ptr, &p.z, 4); ptr += 4;

                if (hasNormals)
                {
                    const glm::vec3& n = data.normals[i];
                    std::memcpy(ptr, &n.x, 4); ptr += 4;
                    std::memcpy(ptr, &n.y, 4); ptr += 4;
                    std::memcpy(ptr, &n.z, 4); ptr += 4;
                }
            }

            file.write(reinterpret_cast<const char*>(buf.data()), (std::streamsize)buf.size());
        }

        // --------------------------------------------------------
        // 3. Pack face data into a contiguous buffer
        //
        // Each triangle on disk: [03][i0][i1][i2]
        //   1 byte  — vertex count as uchar (always 3)
        //  12 bytes — three u32 indices
        //  = 13 bytes per face
        // --------------------------------------------------------

        if (hasFaces)
        {
            constexpr size_t bytesPerFace = 1 + 3 * sizeof(u32); // = 13
            const size_t faceBytes = data.indexBuffer.size() * bytesPerFace;

            std::vector<u8> buf(faceBytes);
            u8* ptr = buf.data();

            for (const glm::uvec3& tri : data.indexBuffer)
            {
                *ptr++ = 3; // vertex count — uchar, always 3 for triangles
                std::memcpy(ptr, &tri.x, 4); ptr += 4;
                std::memcpy(ptr, &tri.y, 4); ptr += 4;
                std::memcpy(ptr, &tri.z, 4); ptr += 4;
            }

            file.write(reinterpret_cast<const char*>(buf.data()), (std::streamsize)buf.size());
        }

        if (!file.good())
        {
            GEOLOGERROR("SavePLY: write error for: " << path);
            return false;
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
                if (!SaveOBJ(path, data)) {
                    GEOLOGERROR("SaveGeometry failed to save OBJ file: " << path);
                }
                return;
            }
            case FileType::PLY:
            {
                if (!SavePLY(path, data)) {
                    GEOLOGERROR("SaveGeometry failed to save PLY file: " << path);
                }
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
