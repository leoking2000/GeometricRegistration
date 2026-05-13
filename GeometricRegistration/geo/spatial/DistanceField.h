#pragma once
#include <unordered_map>
#include <glm/glm.hpp>
#include <geo/geometry/Mesh.h>


namespace geo
{
    bool closestPointToTriangle(
        glm::vec3& out_closestPoint,  f32& out_distance,
        const glm::vec3& query,
        const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
        const glm::vec3& an, const glm::vec3& bn, const glm::vec3& cn,
        const glm::vec3& face_normal,
        f32 maxDist = F32_MAX);

    // Structure to hold the data of the DF cell during DF estimation.
    class DFCell
    {
    public:
        DFCell() = default;
        DFCell(glm::vec3 center, glm::ivec3 coord, const Mesh* mesh)
            : 
            m_center(center), m_coord(coord), m_mesh(mesh), m_distance(F32_MAX)
        {
        }
    public:
        f32 operator () (const glm::vec3& q) const;
        void addTriangle(index_t tri);
    private:
        glm::vec3 m_center;
        glm::ivec3 m_coord;
        f32 m_distance = F32_MAX;
        f32 m_sign = 1.0f;
    private:
        index_t m_tri = INVALID_INDEX;
        bool m_occupied = false;
    private:
        const Mesh* m_mesh;
    };

    struct DistanceFieldParameters
    {
        BBox bounding_box;
        f32 max_distance = 0.0f;
        u32 resolution = 128;
        f32 cell_size = -1.0f;
    };

    //class DistanceField
    //{
    //public:
    //    DistanceField(const DistanceFieldParameters& params);
    //
    //public:
    //    void build(const Mesh& mesh);
    //public:
    //    f32 operator()(const glm::vec3& q) const;
    //private:
    //    void expand();
    //    void compact();
    //private:
    //    BBox m_box;
    //    glm::uvec3 m_dims;
    //    u32 m_resolution;
    //private:
    //    f32 m_epsilon;
    //private:
    //    f32 m_max_dist;
    //    std::unordered_map<u32, f32> m_cells;
    //private:
    //    // temporary data
    //    std::unordered_map<u32, DFCell> m_children;
    //};

    struct GridDescriptor
    {
        BBox bbox;                // bounding box
        glm::uvec3 gridSize{ 0 }; // number of voxels per axis
        f32 voxelSize = 0;        // world units per voxel
    };

    GridDescriptor ComputeGridDescriptor(const BBox& bbox, u32 resolution, f32 padding = 0.05f);

    class SparseVoxelGrid
    {
    public:
        SparseVoxelGrid() = default;
        SparseVoxelGrid(const GridDescriptor& descriptor, f32 defaultValue);
    public:
        // Get value at voxel coordinate (returns default if: outside grid bounds or voxel not stored)
        f32 Get(const glm::uvec3& coord) const;

        // Set value at voxel coordinate (ignored if outside grid bounds)
        void Set(const glm::uvec3& coord, f32 value);

        // Check if coordinate is inside grid bounds
        bool IsInside(const glm::uvec3& coord) const;
    public:
        // Access grid description
        inline const GridDescriptor& GetDescriptor()  const { return m_descriptor; }
        inline const BBox& GetBoundingBox() const { return m_descriptor.bbox; }
    public:
        inline f32  GetDefaultValue() const { return m_defaultValue; }
        inline void SetDefaultValue(f32 value) { m_defaultValue = value; }
    private:
        // Convert 3D coordinate to hash key
        // Pack 3D coordinate into a 64-bit key
        // Uses 21 bits per axis, supports up to ~2 million per dimension
        inline u64 ToKey(const glm::uvec3& coord) const
        {
            return (u64(coord.x) << 42) | (u64(coord.y) << 21) | u64(coord.z);
        }
    private:
        GridDescriptor m_descriptor = {};
        f32 m_defaultValue = F32_MAX;

        // Sparse storage: only stores explicitly set voxels
        std::unordered_map<u64, f32> m_data;
    };

    class NaiveDistanceField
    {
    public:
        NaiveDistanceField() = default;
    public:
        // Build truncated distance field from PointCloud3D
        void Build(const PointCloud3D& cloud, u32 resolution, f32 dTrunc, f32 padding = 0.05f);
    public:
        // Query distance at world-space point
        f32 operator()(const glm::vec3& q) const;
    public:
        SparseVoxelGrid m_grid;
    };

}

