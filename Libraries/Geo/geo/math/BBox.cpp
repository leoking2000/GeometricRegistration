#include <cassert>
#include <algorithm>
#include "BBox.h"

namespace geo
{
    void BBox::MakeEmpty()
    {
        m_min = glm::vec3(F32_MAX, F32_MAX, F32_MAX);
        m_max = glm::vec3(-F32_MAX, -F32_MAX, -F32_MAX);
        m_center = m_size = glm::vec3(0.0f, 0.0f, 0.0f);
    }

    void BBox::Set(const glm::vec3& corner_min, const glm::vec3& corner_max)
    {
        m_min = glm::min(corner_min, corner_max);
        m_max = glm::max(corner_min, corner_max);

        recomputeDerived();
    }

    void BBox::SetSymmetrical(const glm::vec3& center_point, const glm::vec3& size)
    {
        m_size = size;
        m_center = center_point;

        glm::vec3 half = 0.5f * size;

        m_min = center_point - half;
        m_max = center_point + half;
    }

    f32 BBox::MaxSize() const
    {
        return std::max(m_size.x, std::max(m_size.y, m_size.z));
    }

    void BBox::ExpandBy(const glm::vec3& v)
    {
        if (!IsValid())
        {
            Set(v, v);
            return;
        }

        m_min = glm::min(m_min, v);
        m_max = glm::max(m_max, v);

        recomputeDerived();
    }

    void BBox::ExpandBy(const BBox& bbox)
    {
        if (!bbox.IsValid())
            return;

        if (!IsValid())
        {
            *this = bbox;
            return;
        }

        m_min = glm::min(m_min, bbox.m_min);
        m_max = glm::max(m_max, bbox.m_max);

        recomputeDerived();
    }

    void BBox::ExpandByFactor(f32 factor)
    {
        if (!IsValid())
            return;

        glm::vec3 halfExpand = 0.5f * m_size * factor;

        m_min -= halfExpand;
        m_max += halfExpand;

        recomputeDerived();
    }

    void BBox::ExpandByAbsolute(f32 padding)
    {
        if (!IsValid())
            return;

        glm::vec3 p(padding);

        m_min -= p;
        m_max += p;

        recomputeDerived();
    }

    glm::vec3 BBox::Corner(u32 pos) const
    {
        assert(pos < 8);

        return glm::vec3(
            (pos & 1) ? m_max.x : m_min.x,
            (pos & 2) ? m_max.y : m_min.y,
            (pos & 4) ? m_max.z : m_min.z
        );
    }

    void BBox::recomputeDerived()
    {
        m_size = m_max - m_min;
        m_center = 0.5f * (m_min + m_max);
    }
}