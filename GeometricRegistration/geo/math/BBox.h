#pragma once
#include <geo/utils/GeoTypes.h>
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/compatibility.hpp>

namespace geo
{
	// bounding box
	class BBox
	{
	public:
		BBox() { MakeEmpty(); };
		BBox(const glm::vec3& corner_min, const glm::vec3& corner_max) { Set(corner_min, corner_max); };
	public:
		void MakeEmpty();
		void Set(const glm::vec3& corner_min, const glm::vec3& corner_max);
		void SetSymmetrical(const glm::vec3& center_point, const glm::vec3& size);
	public:
		f32              MaxSize() const;
		inline glm::vec3 Size()   const { return m_size; }
		inline glm::vec3 Center() const { return m_center; }
		inline glm::vec3 Min()    const { return m_min; }
		inline glm::vec3 Max()    const { return m_max; }
		inline bool IsValid() const
		{
			// Check finite values (no NaN / inf)
			if (!glm::all(glm::isfinite(m_min)) || !glm::all(glm::isfinite(m_max)))
				return false;

			// Check proper bounds
			return (m_min.x <= m_max.x) &&
				   (m_min.y <= m_max.y) &&
				   (m_min.z <= m_max.z);
		}
	public:
		inline f32 Radius() const  { return glm::sqrt(Radius2()); }
		inline f32 Radius2() const { return (IsValid()) ? 0.25f * (glm::dot(m_size, m_size)) : 0.0f; }
	public:
		// Expands the bounding box to include the given coordinate.
		// If the box is uninitialized, set its m_min and m_max extents to v. */
		void ExpandBy(const glm::vec3& v);

		// Expands this bounding box to include the given bounding box.
		// If this box is uninitialized, set it equal to box.
		void ExpandBy(const BBox& bbox);
	public:
		// Returns true if this bounding box contains the specified coordinate.
		inline bool Contains(const glm::vec3& v) const
		{
			return IsValid() &&
				(v[0] >= m_min[0] && v[0] <= m_max[0]) &&
				(v[1] >= m_min[1] && v[1] <= m_max[1]) &&
				(v[2] >= m_min[2] && v[2] <= m_max[2]);
		}
	public:
		// Returns a specific corner of the bounding box.
		// pos specifies the corner as a number between 0 and 7.
		// Each bit selects an axis, X, Y, or Z from least- to
		// most-significant. Unset bits select the m_minimum value
		// for that axis, and set bits select the m_maximum.
		glm::vec3 Corner(u32 pos) const;
	private:
		glm::vec3 m_size;
		glm::vec3 m_min;
		glm::vec3 m_max;
		glm::vec3 m_center;
	};
}
