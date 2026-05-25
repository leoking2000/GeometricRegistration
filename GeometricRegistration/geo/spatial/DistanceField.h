#pragma once
#include <filesystem>
#include <unordered_map>
#include <glm/glm.hpp>
#include <geo/geometry/Mesh.h>

#define DF_PARALLEL_BUILD

namespace geo
{
    bool closestPointToTriangle(
        glm::vec3& out_closestPoint,  f32& out_distance,
        const glm::vec3& p,
        const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
        const glm::vec3& faceNormal,
        f32 maxDist = F32_MAX);

    bool closestPointToTriangleByIndex(
        glm::vec3& out_closestPoint, f32& out_distance,
        const glm::vec3& p,
        index_t tri, const Mesh& mesh,
        f32 maxDist = F32_MAX);

    // Structure to hold the data of the DF cell during DF estimation.
    class DFCell
    {
    public:
        DFCell() = default;
        DFCell(const glm::vec3& center, const glm::ivec3& coord, const Mesh* mesh)
            : 
            m_center(center), m_coord(coord), m_mesh(mesh),
            m_distance(F32_MAX), m_tri(INVALID_INDEX)
        {}
    public:
        void addTriangle(index_t tri);
        void SetResult(index_t tri, f32 dist);
    public:
        inline bool IsOccupied() const { return m_mesh != nullptr && m_tri != INVALID_INDEX && m_tri < m_mesh->TriangleCount(); };
        inline f32 Distance() const { return m_distance; };
        inline const glm::vec3& Center() const { return m_center; };
        inline const glm::ivec3& Coord() const { return m_coord; };
        inline const index_t& ClosestTriangleIndex() const { return m_tri; }
    private:
        glm::vec3 m_center = glm::vec3(0.0f);
        glm::ivec3 m_coord = glm::ivec3(0);
        f32 m_distance = F32_MAX;
    private:
        index_t m_tri = INVALID_INDEX;
        const Mesh* m_mesh = nullptr;
    };

    struct DistanceFieldParameters
    {
        BBox bounding_box;
        f32 max_distance = 0.0f;
        u32 resolution = 128;
    };

    struct U64Hash {
        inline size_t operator()(u64 k) const noexcept {
            k ^= k >> 33;
            k *= 0xff51afd7ed558ccdULL;
            k ^= k >> 33;
            k *= 0xc4ceb9fe1a85ec53ULL;
            k ^= k >> 33;
            return static_cast<size_t>(k);
        }
    };

    class DistanceField
    {
    public:
        DistanceField() = default;
        DistanceField(const DistanceFieldParameters& params);
    public:
        void Build(const Mesh& mesh);
        f32 operator()(const glm::vec3& q) const;
        inline f32 GetMaxDist() const { return m_max_dist; }
    public:
        // Saves the built SDF to a compact binary file for fast reloading.
        bool Save(const std::filesystem::path& path) const;
        // Loads a previously saved SDF binary. Returns false if file is invalid.
        static bool Load(const std::filesystem::path& path, DistanceField& out);
    private:
        void ExpandSeiral(const Mesh& mesh);
        void ExpandParaller(const Mesh& mesh);
        void computeSignAndCompact(const Mesh& mesh);
    private:
        // Convert 3D coordinate to hash key
        // Pack 3D coordinate into a 64-bit key
        // Uses 21 bits per axis, supports up to ~2 million per dimension
        inline u64 Hash(const glm::uvec3& coord) const
        {
            return (u64(coord.x) << 42) | (u64(coord.y) << 21) | u64(coord.z);
        }
    private:
        BBox m_box;
        glm::ivec3 m_dims{0};
        u32 m_resolution = 0u;
        f32 m_cellSize = 0.0f;
    private:
        f32 m_max_dist = F32_MAX;
        std::unordered_map<u64, DFCell, U64Hash> m_cells;
        std::unordered_map<u64, f32, U64Hash> m_compact_cells;
    };
}
