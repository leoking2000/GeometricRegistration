#include <fstream>
#include <sstream>
#include <algorithm>
#include <cstring>
#include <limits>
#include <map>

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

#include <geo/utils/logging/LogMacros.h>
#include "IOUtils.h"
#include "IOGeometry.h"


namespace geo::io
{
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

        if (ext == ".obj") return FileType::OBJ;
        if (ext == ".ply") return FileType::PLY;

        return FileType::UNKNOWN;
    }

    // ============================================================
    // GeometryDumpData::ToString
    // ============================================================

    std::string GeometryDumpData::ToString() const
    {
        std::stringstream ss;

        auto GeometryTypeToString = [](GeometryType type)
            {
                switch (type)
                {
                case GeometryType::POINT_CLOUD:   return "PointCloud";
                case GeometryType::TRIANGLE_MESH: return "TriangleMesh";
                default:                          return "Unknown";
                }
            };

        ss << "GeometryDumpData\n";
        ss << "----------------\n";

        ss << "File:        " << filePath.filename().string() << '\n';
        ss << "Geometry:    " << GeometryTypeToString(geometryType) << '\n';

        ss << "Vertices:    " << positions.size() << '\n';
        ss << "Normals:     " << normals.size() << '\n';
        ss << "TexCoords:   " << texcoords.size() << '\n';
        ss << "Colors:      " << colors.size() << '\n';
        ss << "Groups:      " << groups.size() << '\n';
        ss << "Triangles:   " << indexBuffer.size() << '\n';
        ss << "Materials:   " << materialsMap.size() << '\n';

        if (bbox.IsValid())
        {
            const glm::vec3 bmin = bbox.Min();
            const glm::vec3 bmax = bbox.Max();
            const glm::vec3 size = bbox.Size();

            ss << '\n';
            ss << "BBox Min:    (" << bmin.x << ", " << bmin.y << ", " << bmin.z << ")\n";
            ss << "BBox Max:    (" << bmax.x << ", " << bmax.y << ", " << bmax.z << ")\n";
            ss << "BBox Size:   (" << size.x << ", " << size.y << ", " << size.z << ")\n";
            ss << "BBox Radius: " << bbox.Radius() << '\n';
        }

        return ss.str();
    }

    // ============================================================
    // Internal helpers
    // ============================================================

    // Compute and store a bounding box from the point set.
    static inline void ComputeBBox(GeometryDumpData& data)
    {
        if (data.positions.empty()) return;

        data.bbox.MakeEmpty();
        for (const glm::vec3& p : data.positions)
        {
            data.bbox.ExpandBy(p);
        }
    }

    // Compute per-vertex normals from triangle soup when normals are absent.
    static void ComputeNormals(GeometryDumpData& data)
    {
        if (data.positions.empty() || data.indexBuffer.empty()) return;

        data.normals.assign(data.positions.size(), glm::vec3(0.0f));

        for (const TriangleIndex& tri : data.indexBuffer)
        {
            const u32 i0 = tri.vertexIndex.x;
            const u32 i1 = tri.vertexIndex.y;
            const u32 i2 = tri.vertexIndex.z;

            if (i0 >= data.positions.size() || i1 >= data.positions.size() || i2 >= data.positions.size())
            {
                continue;
            }

            const glm::vec3& p0 = data.positions[i0];
            const glm::vec3& p1 = data.positions[i1];
            const glm::vec3& p2 = data.positions[i2];

            const glm::vec3 e1 = p1 - p0;
            const glm::vec3 e2 = p2 - p0;
            const glm::vec3 faceNormal = glm::cross(e1, e2);

            data.normals[i0] += faceNormal;
            data.normals[i1] += faceNormal;
            data.normals[i2] += faceNormal;
        }

        for (glm::vec3& n : data.normals)
        {
            n = glm::normalize(n);
        }
    }

    // ============================================================
    // OBJ Loader
    // ============================================================

    static GeometryDumpData LoadOBJ(const std::filesystem::path& path)
    {
        GeometryDumpData data;
        data.filePath = path;
        data.fileType = FileType::OBJ;
        data.geometryType = GeometryType::TRIANGLE_MESH;

        tinyobj::attrib_t                attrib;
        std::vector<tinyobj::shape_t>    shapes;
        std::vector<tinyobj::material_t> materials;

        std::string warn;
        std::string err;

        const bool success = tinyobj::LoadObj(
            &attrib, &shapes, &materials,
            &warn, &err,
            path.string().c_str(),
            path.parent_path().string().c_str(),
            true // triangulate
        );

        if (!warn.empty()) { GEOLOGWARN(warn); }
        if (!err.empty()) { GEOLOGERROR(err); }

        if (!success)
        {
            GEOLOGERROR("Failed to load OBJ: " << path);
            data.geometryType = GeometryType::UNKNOWN;
            data.fileType = FileType::UNKNOWN;
            return data;
        }

        // --------------------------------------------------------
        // Raw attribute buffers
        // --------------------------------------------------------

        data.positions.reserve(attrib.vertices.size() / 3);
        for (size_t i = 0; i + 2 < attrib.vertices.size(); i += 3)
        {
            data.positions.emplace_back(attrib.vertices[i + 0], attrib.vertices[i + 1], attrib.vertices[i + 2]);
        }

        // Load raw OBJ normal buffer (per-attribute, not per-vertex).
        // normalIndex in TriangleIndex refers to these slots.
        if (!attrib.normals.empty())
        {
            data.normals.reserve(attrib.normals.size() / 3);
            for (size_t i = 0; i + 2 < attrib.normals.size(); i += 3)
            {
                data.normals.emplace_back(attrib.normals[i + 0], attrib.normals[i + 1], attrib.normals[i + 2]);
            }
        }

        // Load raw OBJ texcoord buffer.
        if (!attrib.texcoords.empty())
        {
            data.texcoords.reserve(attrib.texcoords.size() / 2);
            for (size_t i = 0; i + 1 < attrib.texcoords.size(); i += 2)
            {
                data.texcoords.emplace_back(attrib.texcoords[i + 0], attrib.texcoords[i + 1]);
            }
        }

        // Load per-vertex colors if tinyobj parsed them
        if (!attrib.colors.empty())
        {
            data.colors.reserve(attrib.colors.size() / 3);
            for (size_t i = 0; i + 2 < attrib.colors.size(); i += 3)
            {
                data.colors.emplace_back(attrib.colors[i + 0], attrib.colors[i + 1], attrib.colors[i + 2]);
            }
        }

        // --------------------------------------------------------
        // Materials
        // --------------------------------------------------------

        for (const tinyobj::material_t& m : materials)
        {
            Material mat;
            mat.name = m.name;
            mat.base_color = glm::vec3(m.diffuse[0], m.diffuse[1], m.diffuse[2]);
            mat.reflectance = 0.05f;
            mat.metallic = m.metallic;
            mat.roughness = m.roughness > 0.0f ? m.roughness : 0.8f;
            mat.texture_file_color = m.diffuse_texname;
            mat.texture_file_normal = m.normal_texname;
            mat.texture_file_mask = m.alpha_texname;

            data.materialsMap[mat.name] = std::move(mat);
        }

        // --------------------------------------------------------
        // Index buffer — preserving OBJ per-face-vertex indices
        // --------------------------------------------------------

        for (const tinyobj::shape_t& shape : shapes)
        {
            if (shape.mesh.num_face_vertices.empty()) continue;

            // Build a TriangleGroup for each shape / material group
            const u32 groupStart = u32(data.indexBuffer.size());

            u32 indexOffset = 0;
            for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); f++)
            {
                const u32 fv = shape.mesh.num_face_vertices[f];

                if (fv != 3)
                {
                    GEOLOGWARN("Skipping non-triangle face in OBJ file (fv=" << fv << ")");
                    indexOffset += fv;
                    continue;
                }

                TriangleIndex tri;

                for (u32 v = 0; v < 3; v++)
                {
                    const tinyobj::index_t& idx = shape.mesh.indices[indexOffset + v];

                    tri.vertexIndex[v] = (idx.vertex_index >= 0) ? u32(idx.vertex_index) : 0u;
                    tri.normalIndex[v] = (idx.normal_index >= 0) ? u32(idx.normal_index) : tri.vertexIndex[v];
                    tri.coordsIndex[v] = (idx.texcoord_index >= 0) ? u32(idx.texcoord_index) : 0u;
                    tri.colorIndex[v] = (idx.vertex_index >= 0) ? u32(idx.vertex_index) : 0u;
                }

                data.indexBuffer.emplace_back(tri);
                indexOffset += 3;
            }

            // Record shape group with material name (if any)
            if (!shape.mesh.material_ids.empty())
            {
                const int matId = shape.mesh.material_ids[0];
                const std::string matName =
                    (matId >= 0 && matId < (int)materials.size()) ? materials[matId].name : std::string();

                TriangleGroup grp;
                grp.start = groupStart;
                grp.length = static_cast<u32>(data.indexBuffer.size()) - groupStart;
                grp.material = matName;
                data.groups.emplace_back(std::move(grp));
            }
        }

        // --------------------------------------------------------
        // Post-process: bbox, and normals if absent
        // --------------------------------------------------------

        ComputeBBox(data);

        if (!data.HasNormals() && data.HasIndices())
        {
            GEOLOGWARN("OBJ has no normals - computing vertex normals in " << GetFileName(path));
            ComputeNormals(data);
        }

        GEOLOGDEBUG("Loaded OBJ: ");
        GEOLOGDEBUG(data.ToString());

        return data;
    }

    // ============================================================
    // PLY Loader
    //
    // Supports:
    //   - ASCII, binary_little_endian, binary_big_endian formats
    //   - Vertex element: x/y/z positions, nx/ny/nz normals (optional),
    //     red/green/blue colors (optional)
    //   - Face element with vertex_indices or vertex_index list property
    //   - Triangle and quad faces (quads split into two triangles)
    //   - Unknown properties skipped safely
    //   - Auto-detects TRIANGLE_MESH vs POINT_CLOUD from face presence
    //   - Does NOT triangulate point clouds or fabricate connectivity
    // ============================================================

    namespace PLY
    {
        enum class PLYFormat { ASCII, BINARY_LE, BINARY_BE, UNKNOWN };
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
            PLYType     type = PLYType::UNKNOWN;
            bool        isList = false;
            PLYType     listCount = PLYType::UNKNOWN;
            PLYType     listElem = PLYType::UNKNOWN;
        };

        struct PLYElement
        {
            std::string              name;
            size_t                   count = 0;
            std::vector<PLYProperty> props;

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
        // SwapBytes = true for big-endian files.
        // All reads use memcpy to avoid strict-aliasing UB.
        // ptr is advanced by the byte-size of the type read.
        // --------------------------------------------------------

        static inline u16 ByteSwap16(u16 v)
        {
            return static_cast<u16>((v >> 8) | (v << 8));
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
            case PLYType::I8: { i8  v = *reinterpret_cast<const i8*>(ptr); ptr += 1; return static_cast<float>(v); }
            case PLYType::U8: { u8  v = *ptr;                               ptr += 1; return static_cast<float>(v); }
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

        template<bool SwapBytes>
        u32 BinaryReadUInt(const u8*& ptr, PLYType t)
        {
            return static_cast<u32>(BinaryReadFloat<SwapBytes>(ptr, t));
        }

        template<bool SwapBytes>
        void BinarySkipProp(const u8*& ptr, const PLYProperty& prop)
        {
            if (prop.isList)
            {
                const u32 count = BinaryReadUInt<SwapBytes>(ptr, prop.listCount);
                ptr += count * PLYTypeBytes(prop.listElem);
            }
            else
            {
                ptr += PLYTypeBytes(prop.type);
            }
        }

    } // namespace PLY

    static GeometryDumpData LoadPLY(const std::filesystem::path& path)
    {
        GeometryDumpData data;
        data.filePath = path;
        data.fileType = FileType::PLY;

        // Open in binary mode — required for binary PLY; harmless for ASCII
        std::ifstream file(path, std::ios::binary);
        if (!file.is_open())
        {
            GEOLOGERROR("Failed to open PLY file: " << path);
            return {};
        }

        // --------------------------------------------------------
        // 1. Parse header
        // --------------------------------------------------------

        PLY::PLYFormat            format = PLY::PLYFormat::UNKNOWN;
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
                if (!line.empty() && line.back() == '\r') line.pop_back();

                std::istringstream ss(line);
                std::string token;
                ss >> token;

                if (token == "end_header")
                {
                    break;
                }
                else if (token == "format")
                {
                    std::string fmt;
                    ss >> fmt;
                    if (fmt == "ascii")               format = PLY::PLYFormat::ASCII;
                    else if (fmt == "binary_little_endian") format = PLY::PLYFormat::BINARY_LE;
                    else if (fmt == "binary_big_endian")    format = PLY::PLYFormat::BINARY_BE;
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
                        prop.isList = true;
                        std::string countTypeStr, elemTypeStr;
                        ss >> countTypeStr >> elemTypeStr >> prop.name;
                        prop.listCount = PLY::StringToPLYType(countTypeStr);
                        prop.listElem = PLY::StringToPLYType(elemTypeStr);
                    }
                    else
                    {
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

        // Locate scalar property indices within the vertex element
        const int xIdx = vertElem->FindProp("x");
        const int yIdx = vertElem->FindProp("y");
        const int zIdx = vertElem->FindProp("z");
        const int nxIdx = vertElem->FindProp("nx");
        const int nyIdx = vertElem->FindProp("ny");
        const int nzIdx = vertElem->FindProp("nz");
        // Colors stored as uchar 0-255 or float 0-1; normalise to [0,1]
        const int rIdx = vertElem->FindProp("red");
        const int gIdx = vertElem->FindProp("green");
        const int bIdx = vertElem->FindProp("blue");

        if (xIdx < 0 || yIdx < 0 || zIdx < 0)
        {
            GEOLOGERROR("PLY vertex element is missing x/y/z properties: " << path);
            return {};
        }

        const bool hasNormals = (nxIdx >= 0 && nyIdx >= 0 && nzIdx >= 0);
        const bool hasColors = (rIdx >= 0 && gIdx >= 0 && bIdx >= 0);

        // Face index list property — accept both common naming conventions
        int faceListPropIdx = -1;
        if (faceElem)
        {
            faceListPropIdx = faceElem->FindProp("vertex_indices");
            if (faceListPropIdx < 0)
                faceListPropIdx = faceElem->FindProp("vertex_index");
        }

        data.positions.reserve(vertElem->count);
        if (hasNormals) data.normals.reserve(vertElem->count);
        if (hasColors)  data.colors.reserve(vertElem->count);
        if (faceElem)   data.indexBuffer.reserve(faceElem->count);

        // Helper: determine whether color channel values need 8-bit normalisation.
        // We check the declared PLY type; if it's U8/I8 we scale to [0,1].
        const bool colorNeedsNorm = (rIdx >= 0) &&
            (vertElem->props[rIdx].type == PLY::PLYType::U8 ||
                vertElem->props[rIdx].type == PLY::PLYType::I8);

        // --------------------------------------------------------
        // 3. Read data
        // --------------------------------------------------------

        if (format == PLY::PLYFormat::ASCII)
        {
            // Each line holds all property values for one vertex, in declaration order.
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

                const int maxPosIdx = std::max({ xIdx, yIdx, zIdx });
                if ((int)tokens.size() <= maxPosIdx) continue;

                data.positions.emplace_back(
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

                if (hasColors)
                {
                    const int maxCIdx = std::max({ rIdx, gIdx, bIdx });
                    if ((int)tokens.size() > maxCIdx)
                    {
                        float r = std::stof(tokens[rIdx]);
                        float g = std::stof(tokens[gIdx]);
                        float b = std::stof(tokens[bIdx]);
                        if (colorNeedsNorm)
                        {
                            r /= 255.0f;
                            g /= 255.0f;
                            b /= 255.0f;
                        }
                        data.colors.emplace_back(r, g, b);
                    }
                }
            }

            // ASCII face read
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

                        TriangleIndex tri;
                        tri.vertexIndex = glm::uvec3(a, b, c);
                        // PLY has no separate normal/texcoord index buffers;
                        // vertex, normal, and color attributes are stored per-vertex.
                        tri.normalIndex = tri.vertexIndex;
                        tri.colorIndex = tri.vertexIndex;
                        tri.coordsIndex = glm::uvec3(0);
                        data.indexBuffer.emplace_back(tri);
                    }
                    else if (count == 4)
                    {
                        u32 a, b, c, d;
                        ss >> a >> b >> c >> d;

                        TriangleIndex t0, t1;
                        t0.vertexIndex = glm::uvec3(a, b, c);
                        t0.normalIndex = t0.vertexIndex;
                        t0.colorIndex = t0.vertexIndex;
                        t0.coordsIndex = glm::uvec3(0);

                        t1.vertexIndex = glm::uvec3(a, c, d);
                        t1.normalIndex = t1.vertexIndex;
                        t1.colorIndex = t1.vertexIndex;
                        t1.coordsIndex = glm::uvec3(0);

                        data.indexBuffer.emplace_back(t0);
                        data.indexBuffer.emplace_back(t1);
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
            // --------------------------------------------------------
            // Binary read (little-endian or big-endian)
            //
            // Slurp remaining bytes into memory and walk a raw pointer —
            // avoids repeated istream overhead for large scan files.
            // --------------------------------------------------------

            std::vector<u8> raw(
                (std::istreambuf_iterator<char>(file)),
                std::istreambuf_iterator<char>()
            );
            const u8* ptr = raw.data();
            const u8* end = ptr + raw.size();

            const bool isBigEndian = (format == PLY::PLYFormat::BINARY_BE);

            // Dispatch to the right endian template at load-time, keeping the
            // inner loop branch-free.
            auto readFloat = [&](PLY::PLYType t) -> float
                {
                    return isBigEndian ? PLY::BinaryReadFloat<true>(ptr, t) : PLY::BinaryReadFloat<false>(ptr, t);
                };

            auto readUInt = [&](PLY::PLYType t) -> u32
                {
                    return isBigEndian ? PLY::BinaryReadUInt<true>(ptr, t) : PLY::BinaryReadUInt<false>(ptr, t);
                };

            auto skipProp = [&](const PLY::PLYProperty& prop)
                {
                    isBigEndian ? PLY::BinarySkipProp<true>(ptr, prop) : PLY::BinarySkipProp<false>(ptr, prop);
                };

            // Binary vertex read
            for (size_t i = 0; i < vertElem->count; i++)
            {
                if (ptr >= end) break;

                float x = 0, y = 0, z = 0;
                float nx = 0, ny = 0, nz = 0;
                float r = 0, g = 0, b = 0;

                for (int p = 0; p < (int)vertElem->props.size(); p++)
                {
                    const PLY::PLYProperty& prop = vertElem->props[p];

                    if (prop.isList)
                    {
                        skipProp(prop);
                    }
                    else if (p == xIdx)  x = readFloat(prop.type);
                    else if (p == yIdx)  y = readFloat(prop.type);
                    else if (p == zIdx)  z = readFloat(prop.type);
                    else if (p == nxIdx) nx = readFloat(prop.type);
                    else if (p == nyIdx) ny = readFloat(prop.type);
                    else if (p == nzIdx) nz = readFloat(prop.type);
                    else if (p == rIdx)  r = readFloat(prop.type);
                    else if (p == gIdx)  g = readFloat(prop.type);
                    else if (p == bIdx)  b = readFloat(prop.type);
                    else                 ptr += PLY::PLYTypeBytes(prop.type); // unknown scalar
                }

                data.positions.emplace_back(x, y, z);

                if (hasNormals)
                    data.normals.emplace_back(nx, ny, nz);

                if (hasColors)
                {
                    if (colorNeedsNorm) { r /= 255.0f; g /= 255.0f; b /= 255.0f; }
                    data.colors.emplace_back(r, g, b);
                }
            }

            // Binary face read
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
                            const u32 count = readUInt(prop.listCount);

                            if (count == 3)
                            {
                                const u32 a = readUInt(prop.listElem);
                                const u32 b = readUInt(prop.listElem);
                                const u32 c = readUInt(prop.listElem);

                                TriangleIndex tri;
                                tri.vertexIndex = glm::uvec3(a, b, c);
                                tri.normalIndex = tri.vertexIndex;
                                tri.colorIndex = tri.vertexIndex;
                                tri.coordsIndex = glm::uvec3(0);
                                data.indexBuffer.push_back(tri);
                            }
                            else if (count == 4)
                            {
                                const u32 a = readUInt(prop.listElem);
                                const u32 b = readUInt(prop.listElem);
                                const u32 c = readUInt(prop.listElem);
                                const u32 d = readUInt(prop.listElem);

                                TriangleIndex t0, t1;
                                t0.vertexIndex = glm::uvec3(a, b, c);
                                t0.normalIndex = t0.vertexIndex;
                                t0.colorIndex = t0.vertexIndex;
                                t0.coordsIndex = glm::uvec3(0);

                                t1.vertexIndex = glm::uvec3(a, c, d);
                                t1.normalIndex = t1.vertexIndex;
                                t1.colorIndex = t1.vertexIndex;
                                t1.coordsIndex = glm::uvec3(0);

                                data.indexBuffer.push_back(t0);
                                data.indexBuffer.push_back(t1);
                            }
                            else
                            {
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
        // 4. Determine geometry type — strictly from file contents.
        //    A PLY with no face element is always a point cloud;
        //    we never fabricate connectivity.
        // --------------------------------------------------------

        data.geometryType = data.HasIndices() ? GeometryType::TRIANGLE_MESH : GeometryType::POINT_CLOUD;

        // --------------------------------------------------------
        // 5. Post-process: bbox; compute normals for meshes if absent
        // --------------------------------------------------------

        ComputeBBox(data);

        if (!data.HasNormals() && data.geometryType == GeometryType::TRIANGLE_MESH)
        {
            GEOLOGWARN("PLY mesh has no normals - computing vertex normals in " << GetFileName(path));
            ComputeNormals(data);
        }

        GEOLOGDEBUG("Loaded PLY: ");
        GEOLOGDEBUG(data.ToString());

        return data;
    }

    // ============================================================
    // OBJ Saver
    // ============================================================

    static bool SaveOBJ(const std::filesystem::path& path, const GeometryDumpData& data)
    {
        std::ofstream file(path);
        if (!file.is_open())
        {
            GEOLOGERROR("Failed to open OBJ for writing: " << path);
            return false;
        }

        file << "# Generated by geo::io\n";

        // --------------------------------------------------------
        // Positions
        // --------------------------------------------------------

        for (const glm::vec3& p : data.positions)
        {
            file << "v " << p.x << " " << p.y << " " << p.z << "\n";
        }

        // --------------------------------------------------------
        // Texture coordinates
        // --------------------------------------------------------

        for (const glm::vec2& uv : data.texcoords)
        {
            file << "vt " << uv.x << " " << uv.y << "\n";
        }

        // --------------------------------------------------------
        // Normals
        // --------------------------------------------------------

        for (const glm::vec3& n : data.normals)
        {
            file << "vn " << n.x << " " << n.y << " " << n.z << "\n";
        }

        // --------------------------------------------------------
        // Materials
        // --------------------------------------------------------

        if (!data.materialsMap.empty())
        {
            // Write a companion .mtl alongside the .obj
            const std::filesystem::path mtlPath = path.parent_path() / (path.stem().string() + ".mtl");
            std::ofstream mtlFile(mtlPath);
            if (mtlFile.is_open())
            {
                file << "mtllib " << mtlPath.filename().string() << "\n";

                for (const auto& [name, mat] : data.materialsMap)
                {
                    mtlFile << "newmtl " << mat.name << "\n";
                    mtlFile << "Kd " << mat.base_color.r << " " << mat.base_color.g << " " << mat.base_color.b << "\n";
                    mtlFile << "Pr " << mat.roughness << "\n";
                    mtlFile << "Pm " << mat.metallic << "\n";
                    if (!mat.texture_file_color.empty())
                        mtlFile << "map_Kd " << mat.texture_file_color << "\n";
                    if (!mat.texture_file_normal.empty())
                        mtlFile << "norm " << mat.texture_file_normal << "\n";
                    if (!mat.texture_file_mask.empty())
                        mtlFile << "map_d " << mat.texture_file_mask << "\n";
                    mtlFile << "\n";
                }
            }
            else
            {
                GEOLOGWARN("SaveOBJ: could not write MTL file: " << mtlPath);
            }
        }

        // --------------------------------------------------------
        // Triangles — groups partition the index buffer by material
        //
        // We reconstruct per-group "usemtl" and "g" blocks when groups
        // are present, otherwise write all faces as a single block.
        //
        // OBJ face syntax (1-based):
        //   f v/vt/vn  v/vt/vn  v/vt/vn
        // --------------------------------------------------------

        const bool hasNormals = data.HasNormals();
        const bool hasTexcoord = data.HasCoords();

        auto writeFace = [&](const TriangleIndex& tri)
            {
                file << "f ";
                for (int v = 0; v < 3; v++)
                {
                    const u32 vi = tri.vertexIndex[v] + 1; // OBJ is 1-based
                    const u32 ti = tri.coordsIndex[v] + 1;
                    const u32 ni = tri.normalIndex[v] + 1;

                    if (hasNormals && hasTexcoord)
                        file << vi << "/" << ti << "/" << ni;
                    else if (hasNormals)
                        file << vi << "//" << ni;
                    else if (hasTexcoord)
                        file << vi << "/" << ti;
                    else
                        file << vi;

                    if (v < 2) file << " ";
                }
                file << "\n";
            };

        if (!data.groups.empty())
        {
            for (const TriangleGroup& grp : data.groups)
            {
                if (!grp.material.empty())
                {
                    file << "usemtl " << grp.material << "\n";
                    file << "g " << grp.material << "\n";
                }

                const u32 end = grp.start + grp.length;
                for (u32 i = grp.start; i < end && i < (u32)data.indexBuffer.size(); i++)
                {
                    writeFace(data.indexBuffer[i]);
                }
            }
        }
        else
        {
            for (const TriangleIndex& tri : data.indexBuffer)
            {
                writeFace(tri);
            }
        }

        if (!file.good())
        {
            GEOLOGERROR("SaveOBJ: write error: " << path);
            return false;
        }

        GEOLOGDEBUG("Saved OBJ: ");
        GEOLOGDEBUG(data.ToString());

        return true;
    }

    // ============================================================
    // PLY Saver
    //
    // Writes binary little-endian PLY.  Includes normals and colors
    // when present.  Mesh topology (faces) written only when indices exist.
    // ============================================================

    static bool SavePLY(const std::filesystem::path& path, const GeometryDumpData& data)
    {
        if (data.positions.empty())
        {
            GEOLOGERROR("SavePLY: no points to write");
            return false;
        }

        // Binary mode is required — text mode on Windows translates '\n' to '\r\n'
        // inside the data section, corrupting binary values.
        std::ofstream file(path, std::ios::binary);
        if (!file.is_open())
        {
            GEOLOGERROR("SavePLY: failed to open file for writing: " << path);
            return false;
        }

        const bool hasNormals = data.HasNormals();
        const bool hasColors = data.HasColors();
        const bool hasFaces = data.HasIndices();

        // --------------------------------------------------------
        // 1. Write ASCII header
        //
        // The header is always ASCII text, even in binary PLY files.
        // Declared property order must exactly match the binary layout below.
        // --------------------------------------------------------

        file << "ply\n";
        file << "format binary_little_endian 1.0\n";
        file << "comment Created by geo::io\n";
        file << "element vertex " << data.positions.size() << "\n";
        file << "property float x\n";
        file << "property float y\n";
        file << "property float z\n";

        if (hasNormals)
        {
            file << "property float nx\n";
            file << "property float ny\n";
            file << "property float nz\n";
        }

        if (hasColors)
        {
            file << "property uchar red\n";
            file << "property uchar green\n";
            file << "property uchar blue\n";
        }

        if (hasFaces)
        {
            file << "element face " << data.indexBuffer.size() << "\n";
            file << "property list uchar uint vertex_indices\n";
        }

        file << "end_header\n";

        // --------------------------------------------------------
        // 2. Pack vertex data into a contiguous buffer and flush once
        //
        // Layout per vertex (all floats, little-endian native):
        //   x y z [nx ny nz] [r g b as uchar]
        // --------------------------------------------------------

        {
            const size_t floatsPerVertex = 3
                + (hasNormals ? 3 : 0);
            const size_t bytesPerVertex = floatsPerVertex * sizeof(float)
                + (hasColors ? 3 : 0); // 3 uchars for RGB
            const size_t totalBytes = data.positions.size() * bytesPerVertex;

            std::vector<u8> buf(totalBytes);
            u8* ptr = buf.data();

            for (size_t i = 0; i < data.positions.size(); i++)
            {
                const glm::vec3& p = data.positions[i];
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

                if (hasColors)
                {
                    const glm::vec3& c = data.colors[i];
                    *ptr++ = static_cast<u8>(std::min(c.r * 255.0f + 0.5f, 255.0f));
                    *ptr++ = static_cast<u8>(std::min(c.g * 255.0f + 0.5f, 255.0f));
                    *ptr++ = static_cast<u8>(std::min(c.b * 255.0f + 0.5f, 255.0f));
                }
            }

            file.write(reinterpret_cast<const char*>(buf.data()), static_cast<std::streamsize>(buf.size()));
        }

        // --------------------------------------------------------
        // 3. Pack face data into a contiguous buffer
        //
        // Per triangle on disk: [0x03][i0][i1][i2]
        //   1 byte  — vertex count as uchar (always 3)
        //  12 bytes — three u32 vertex indices
        //  = 13 bytes per face
        //
        // We write only the vertex position indices (vertexIndex).
        // --------------------------------------------------------

        if (hasFaces)
        {
            constexpr size_t bytesPerFace = 1 + 3 * sizeof(u32); // 13 bytes
            const size_t faceBytes = data.indexBuffer.size() * bytesPerFace;

            std::vector<u8> buf(faceBytes);
            u8* ptr = buf.data();

            for (const TriangleIndex& tri : data.indexBuffer)
            {
                *ptr++ = 3;
                std::memcpy(ptr, &tri.vertexIndex.x, 4); ptr += 4;
                std::memcpy(ptr, &tri.vertexIndex.y, 4); ptr += 4;
                std::memcpy(ptr, &tri.vertexIndex.z, 4); ptr += 4;
            }

            file.write(reinterpret_cast<const char*>(buf.data()), static_cast<std::streamsize>(buf.size()));
        }

        if (!file.good())
        {
            GEOLOGERROR("SavePLY: write error: " << path);
            return false;
        }

        GEOLOGDEBUG("Saved PLY: ");
        GEOLOGDEBUG(data.ToString());

        return true;
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

        switch (GetFileType(path))
        {
        case FileType::OBJ: return LoadOBJ(path);
        case FileType::PLY: return LoadPLY(path);
        default:
            GEOLOGERROR("Unsupported geometry file type: " << path);
            return {};
        }
    }

    // ============================================================
    // Geometry Saving
    // ============================================================

    void SaveGeometry(const std::filesystem::path& path, const GeometryDumpData& data)
    {
        switch (GetFileType(path))
        {
        case FileType::OBJ:
            if (!SaveOBJ(path, data))
                GEOLOGERROR("SaveGeometry failed to save OBJ file: " << path);
            return;

        case FileType::PLY:
            if (!SavePLY(path, data))
                GEOLOGERROR("SaveGeometry failed to save PLY file: " << path);
            return;

        default:
            GEOLOGERROR("SaveGeometry: unsupported file type: " << path);
            return;
        }
    }

    // Key: packed (vi, ni, ti, ci) tuple
    struct VertexKey
    {
        u32 vi, ni, ti, ci;
        bool operator==(const VertexKey& o) const
        {
            return vi == o.vi && ni == o.ni && ti == o.ti && ci == o.ci;
        }
    };

    struct KeyHash
    {
        size_t operator()(const VertexKey& k) const
        {
            // FNV-inspired mixing
            size_t h = 2166136261u;
            auto mix = [&](u32 v) { h ^= v; h *= 16777619u; };
            mix(k.vi); mix(k.ni); mix(k.ti); mix(k.ci);
            return h;
        }
    };

    //// ============================================================
    //// MakePointCloud
    ////
    //// Extracts a flat PointCloud from a GeometryDumpData.
    //// Works for both POINT_CLOUD and TRIANGLE_MESH sources;
    //// in the mesh case the vertex positions are simply reused.
    //// ============================================================

    PointCloudData MakePointCloud(const GeometryDumpData& data)
    {
        PointCloudData pc;

        // -------------------------------------------------------
        // Case 1: true point cloud — attributes are already stored
        // one-per-point and aligned; copy directly.
        // -------------------------------------------------------
        if (!data.HasIndices())
        {
            pc.points = data.positions;
            pc.normals = data.normals;
            pc.colors = data.colors;
            return pc;
        }

        // -------------------------------------------------------
        // Case 2: triangle mesh.
        //
        // OBJ uses three independent index channels (vertexIndex,
        // normalIndex, colorIndex), so the raw attribute arrays are
        // NOT aligned — data.normals[i] is NOT the normal for
        // data.points[i].  We must flatten by the combined key
        // (vi, ni, ci) so that after flattening every index i
        // refers to the same geometric corner across all three arrays.
        //
        // A cube is the canonical example:
        //   8 positions × 3 distinct normals per corner = 24 unique
        //   (vi, ni) keys.  Naïve per-triangle expansion produces 36
        //   (6 faces × 2 triangles × 3 verts); deduplication here
        //   collapses that back to the correct 24.
        //
        // Algorithm: walk every triangle corner, hash its (vi, ni, ci)
        // tuple, and emit a new entry only when the tuple is new.
        // -------------------------------------------------------

        const glm::vec3 defaultNormal(0.0f, 1.0f, 0.0f);
        const glm::vec3 defaultColor(1.0f, 1.0f, 1.0f);

        std::unordered_set<VertexKey, KeyHash> seen;
        seen.reserve(data.indexBuffer.size() * 3);

        for (const TriangleIndex& tri : data.indexBuffer)
        {
            for (int v = 0; v < 3; v++)
            {
                const u32 vi = tri.vertexIndex[v];
                const u32 ni = tri.normalIndex[v];
                const u32 ti = tri.normalIndex[v];
                const u32 ci = tri.colorIndex[v];

                // Skip corners we have already emitted
                if (seen.count(VertexKey{ vi, ni, ti, ci })) continue;

                // insert() returns false in .second if key was already present
                if (!seen.insert(VertexKey{ vi, ni, ti, ci }).second) continue;

                pc.points.emplace_back(vi < data.positions.size() ? data.positions[vi] : glm::vec3(0.0f));
                pc.normals.emplace_back(ni < data.normals.size() ? data.normals[ni] : defaultNormal);
                pc.colors.emplace_back(ci < data.colors.size() ? data.colors[ci] : defaultColor);
            }
        }

        return pc;
    }

    //// ============================================================
    //// MakeMeshData
    ////
    //// Builds a flat, indexed vertex buffer suitable for GPU upload.
    //// Each unique (position, normal, texcoord, color) combination
    //// in the index buffer becomes one Vertex entry; duplicate
    //// combinations are collapsed via a hash map.
    ////
    //// If normal / texcoord / color arrays are absent, zero values
    //// are used as sensible defaults.
    //// ============================================================

    MeshData MakeMeshData(const GeometryDumpData& data)
    {
        MeshData mesh;

        if (data.indexBuffer.empty())
            return mesh;

        std::unordered_map<VertexKey, u32, KeyHash> cache;
        cache.reserve(data.indexBuffer.size() * 3);
        mesh.indexBuffer.reserve(data.indexBuffer.size() * 3);

        for (const TriangleIndex& tri : data.indexBuffer)
        {
            for (int v = 0; v < 3; v++)
            {
                const u32 vi = tri.vertexIndex[v];
                const u32 ni = tri.normalIndex[v];
                const u32 ti = tri.coordsIndex[v];
                const u32 ci = tri.colorIndex[v];

                const VertexKey key{ vi, ni, ti, ci };
                auto it = cache.find(key);

                if (it != cache.end())
                {
                    mesh.indexBuffer.emplace_back(it->second);
                }
                else
                {
                    VertexData vtx;

                    vtx.point = (vi < data.positions.size()) ? data.positions[vi] : glm::vec3(0.0f);

                    vtx.normal = (ni < data.normals.size()) ? data.normals[ni] : glm::vec3(0.0f, 1.0f, 0.0f);

                    vtx.texCoord = (ti < data.texcoords.size()) ? data.texcoords[ti] : glm::vec2(0.0f);

                    vtx.color = (ci < data.colors.size()) ? data.colors[ci] : glm::vec3(1.0f);

                    const u32 newIndex = static_cast<u32>(mesh.vertexBuffer.size());
                    mesh.vertexBuffer.emplace_back(vtx);
                    mesh.indexBuffer.emplace_back(newIndex);
                    cache[key] = newIndex;
                }
            }
        }

        return mesh;
    }

}
