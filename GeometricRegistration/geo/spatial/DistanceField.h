#pragma once
#include <unordered_map>
#include <glm/glm.hpp>
#include <geo/geometry/PointCloud3D.h>


namespace geo
{
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
        inline const BBox&           GetBoundingBox() const { return m_descriptor.bbox; }
    public:
        inline f32  GetDefaultValue() const    { return m_defaultValue; }
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

    class DistanceField
    {
    public:
        DistanceField() = default;
    public:
        // Build truncated distance field from PointCloud3D
        void Build(const PointCloud3D& cloud, u32 resolution, f32 dTrunc, f32 padding = 0.05f);
    public:
        // Query distance at world-space point
        f32 operator()(const glm::vec3& q) const;
    private:
        SparseVoxelGrid m_grid;
    };

}

