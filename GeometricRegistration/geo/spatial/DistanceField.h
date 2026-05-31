#pragma once
#include <filesystem>
#include <unordered_map>
#include <glm/glm.hpp>
#include <geo/geometry/Mesh.h>

#define DF_PARALLEL_BUILD

namespace geo
{
    // Computes the closest point on a triangle (A, B, C) to point p.
    //
    // Outputs:
    // - out_closestPoint : closest point on triangle surface
    // - out_distance     : Euclidean distance to p
    //
    // Returns:
    // - true if a valid closest point was computed (within maxDist)
    bool closestPointToTriangle(
        glm::vec3& out_closestPoint,  f32& out_distance,
        const glm::vec3& p,
        const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
        const glm::vec3& faceNormal,
        f32 maxDist = F32_MAX);

    // Same as closestPointToTriangle but operates on a mesh triangle index.
    bool closestPointToTriangleByIndex(
        glm::vec3& out_closestPoint, f32& out_distance,
        const glm::vec3& p,
        index_t tri, const Mesh& mesh,
        f32 maxDist = F32_MAX);

    // Represents a single voxel/cell in the distance field grid.
    //
    // Each cell stores:
    // - geometric center position
    // - grid coordinate
    // - closest triangle index
    // - distance to closest surface
    //
    // Used during construction of the distance field before compacting.
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
        void addTriangle(index_t tri);  // Adds a candidate triangle for proximity evaluation.
        void SetResult(index_t tri, f32 dist); // Stores the closest triangle and its distance result.
    public:
        // Returns true if this cell has been assigned a valid closest triangle.
        inline bool IsOccupied() const { return m_mesh != nullptr && m_tri != INVALID_INDEX && m_tri < m_mesh->TriangleCount(); };
        inline f32 Distance() const { return m_distance; }; // Returns stored distance to closest surface.
        inline const glm::vec3& Center() const { return m_center; }; // Returns voxel center position.
        inline const glm::ivec3& Coord() const { return m_coord; }; // Returns grid coordinate of this cell.
        inline const index_t& ClosestTriangleIndex() const { return m_tri; } // Returns index of closest triangle in the mesh.
    private:
        glm::vec3 m_center = glm::vec3(0.0f);
        glm::ivec3 m_coord = glm::ivec3(0);
        f32 m_distance = F32_MAX;
    private:
        index_t m_tri = INVALID_INDEX;
        const Mesh* m_mesh = nullptr;
    };

    // Distance field construction parameters
    struct DistanceFieldParameters
    {
        BBox bounding_box;
        f32 max_distance = 0.0f;
        u32 resolution = 128;
    };

    // Hash helper for sparce voxel storage
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

    // Sparse voxel distance field (SDF-like structure).
    class DistanceField
    {
    public:
        DistanceField() = default;
        DistanceField(const DistanceFieldParameters& params);
    public:
        void Build(const Mesh& mesh); // Builds the distance field from a triangle mesh.
        f32 operator()(const glm::vec3& q) const; // Evaluates signed distance at query point q.
        inline f32 GetMaxDist() const { return m_max_dist; } // Returns maximum truncation distance used during construction.
        inline f32 GetCellSize() const { return m_cellSize; } // Returns the cellSize
    public:
        // Saves the built SDF to a compact binary file for fast reloading.
        bool Save(const std::filesystem::path& path) const;
        // Loads a previously saved SDF binary. Returns false if file is invalid.
        static bool Load(const std::filesystem::path& path, DistanceField& out);
    private:
        void ExpandSeiral(const Mesh& mesh); // Expands grid sequentially (baseline build method).
        //void ExpandParaller(const Mesh& mesh);
        void computeSignAndCompact(const Mesh& mesh); // Computes sign (inside/outside) and compacts grid into fast lookup form.
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
        std::unordered_map<u64, DFCell, U64Hash> m_cells; // Full voxel storage during construction
        std::unordered_map<u64, f32, U64Hash> m_compact_cells; // Compact representation used at query time
    };
}
