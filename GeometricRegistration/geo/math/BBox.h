#pragma once
#include <geo/utils/GeoTypes.h>
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/compatibility.hpp>

namespace geo
{
	// Axis-Aligned Bounding Box (AABB) in 3D space.
	// Stores minimum/maximum corners along with derived properties
	// such as size and center.
	class BBox final
	{
	public:
		// Default constructor: initializes as an empty/invalid box
		BBox() { MakeEmpty(); };
		// Constructs a bounding box from min/max corners
		BBox(const glm::vec3& corner_min, const glm::vec3& corner_max) { Set(corner_min, corner_max); };
	public:
		// Resets the bounding box to an empty/invalid state
		void MakeEmpty();
		// Sets the bounding box from minimum and maximum corners
		void Set(const glm::vec3& corner_min, const glm::vec3& corner_max);
		// Sets the bounding box symmetrically around a center point
		// @param center_point Center of the box
		// @param size Full size (extent) along each axis
		void SetSymmetrical(const glm::vec3& center_point, const glm::vec3& size);
	public:
		f32              MaxSize() const;                     // Returns the largest dimension (max of width, height, depth)
		inline glm::vec3 Size()   const { return m_size; }    // Returns the size (extent) of the box along each axis
		inline glm::vec3 Center() const { return m_center; }  // Returns the center point of the box
		inline glm::vec3 Min()    const { return m_min; }     // Returns the minimum corner (lower bounds)
		inline glm::vec3 Max()    const { return m_max; }     // Returns the maximum corner (upper bounds)

		// Validity check:
		// - Ensures values are finite (no NaN or infinity)
		// - Ensures min <= max for all axe
		inline bool IsValid() const
		{
			// Check finite values (no NaN / inf)
			if (!glm::all(glm::isfinite(m_min)) || !glm::all(glm::isfinite(m_max)))
				return false;

			// Check proper bounds
			return (m_min.x <= m_max.x) && (m_min.y <= m_max.y) && (m_min.z <= m_max.z);
		}
	public:
		// Returns the radius of the bounding sphere enclosing the box
		inline f32 Radius() const  { return glm::sqrt(Radius2()); }

		// Returns squared radius of the bounding sphere
		// Computed as 1/4 of the squared diagonal length
		inline f32 Radius2() const { return (IsValid()) ? 0.25f * (glm::dot(m_size, m_size)) : 0.0f; }
	public:
		// Expands the bounding box to include a point
		// If the box is invalid/uninitialized, it becomes a single-point box
		void ExpandBy(const glm::vec3& v);

		// Expands the bounding box to include another bounding box
		// If this box is invalid/uninitialized, it copies the input box
		void ExpandBy(const BBox& bbox);

		// Expands the bounding box by a relative factor
		// factor = 0.1 expands each side by 10% of current size
		// (total dimension increases by 20%)
		// No effect if the box is invalid
		void ExpandByFactor(f32 factor);

		// Expands the bounding box by an absolute padding (world units)
		// Padding is applied equally in all directions
		// No effect if the box is invalid
		void ExpandByAbsolute(f32 padding);
	public:
		// Checks whether a point lies inside (or on the boundary of) the box
		inline bool Contains(const glm::vec3& v) const
		{
			return IsValid() &&
				(v[0] >= m_min[0] && v[0] <= m_max[0]) &&
				(v[1] >= m_min[1] && v[1] <= m_max[1]) &&
				(v[2] >= m_min[2] && v[2] <= m_max[2]);
		}
	public:
		// Returns one of the 8 corners of the bounding box
		//
		// @param pos Integer in [0, 7] encoding which corner:
		// - Bit 0 (LSB): X axis (0 = min.x, 1 = max.x)
		// - Bit 1:       Y axis (0 = min.y, 1 = max.y)
		// - Bit 2:       Z axis (0 = min.z, 1 = max.z)
		glm::vec3 Corner(u32 pos) const;
	private:
		// Recomputes cached derived properties from m_min and m_max:
		// - m_size   = extent (max - min)
		// - m_center = midpoint of the box
		void recomputeDerived();
	private:
		glm::vec3 m_min;       // Minimum corner (lower bounds)    
		glm::vec3 m_max;       // Maximum corner (upper bounds)
		glm::vec3 m_size;      // Cached size (max - min)
		glm::vec3 m_center;    // Cached center point
	};
}
