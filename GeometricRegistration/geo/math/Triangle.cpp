#include "Triangle.h"

namespace geo
{
    // Returns unit normal of triangle (a,b,c)
    // Right-hand rule: (b - a) x (c - a)
    glm::vec3 TriangleNormal(const glm::vec3& a,
        const glm::vec3& b,
        const glm::vec3& c)
    {
        const glm::vec3 e1 = b - a;
        const glm::vec3 e2 = c - a;

        const glm::vec3 n = glm::cross(e1, e2);
        const float len = glm::length(n);

        if (len <= std::numeric_limits<float>::epsilon())
            return glm::vec3(0.0f, 0.0f, 0.0f);

        return n / len;
    }

    // Triangle area = 0.5 * || (b-a) x (c-a) ||
    float TriangleArea(const glm::vec3& a,
        const glm::vec3& b,
        const glm::vec3& c)
    {
        const glm::vec3 e1 = b - a;
        const glm::vec3 e2 = c - a;

        return 0.5f * glm::length(glm::cross(e1, e2));
    }

    // Centroid (uniform area triangle)
    glm::vec3 TriangleCentroid(const glm::vec3& a,
        const glm::vec3& b,
        const glm::vec3& c)
    {
        return (a + b + c) / 3.0f;
    }


    bool closestPointToTriangle(glm::vec3& out_closestPoint, f32& out_distance,
        const glm::vec3& p,
        const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
        const glm::vec3& faceNormal,
        f32 maxDist)
    {
        glm::vec3 ab = b - a;
        glm::vec3 ac = c - a;
        glm::vec3 ap = p - a;

        // 1. Calculate distance from the triangle plane and Early Exit
        f32 distToPlane = glm::dot(p - a, faceNormal);
        if (glm::abs(distToPlane) >= maxDist)
        {
            return false;
        }

        // 2. Vertex Region A
        f32 d1 = glm::dot(ab, ap);
        f32 d2 = glm::dot(ac, ap);
        if (d1 <= 0.0f && d2 <= 0.0f)
        {
            f32 d = glm::distance(p, a);

            if (d >= maxDist) {
                return false;
            }

            out_closestPoint = a;
            out_distance = d;
            return true;
        }

        // 3. Vertex Region B
        glm::vec3 bp = p - b;
        f32 d3 = glm::dot(ab, bp);
        f32 d4 = glm::dot(ac, bp);
        if (d3 >= 0.0f && d4 <= d3) {

            float d = glm::distance(p, b);

            if (d >= maxDist) {
                return false;
            }

            out_closestPoint = b;
            out_distance = d;
            return true;
        }

        // 4. Edge Region AB
        f32 vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
            f32 v = d1 / (d1 - d3);
            glm::vec3 closest = a + v * ab;
            float d = glm::distance(p, closest);

            if (d >= maxDist) {
                return false;
            }

            out_closestPoint = closest;
            out_distance = d;
            return true;
        }

        // 5. Vertex Region C
        glm::vec3 cp = p - c;
        f32 d5 = glm::dot(ab, cp);
        f32 d6 = glm::dot(ac, cp);
        if (d6 >= 0.0f && d5 <= d6) {
            f32 d = glm::distance(p, c);

            if (d >= maxDist) {
                return false;
            }

            out_closestPoint = c;
            out_distance = d;
            return true;
        }

        // 6. Edge Region AC
        f32 vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
            f32 w = d2 / (d2 - d6);
            glm::vec3 closest = a + w * ac;
            f32 d = glm::distance(p, closest);

            if (d >= maxDist) {
                return false;
            }

            out_closestPoint = closest;
            out_distance = d;
            return true;
        }

        // 7. Edge Region BC
        f32 va = d3 * d6 - d5 * d4;
        if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
            f32 w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            glm::vec3 closest = b + w * (c - b);
            float d = glm::distance(p, closest);

            if (d >= maxDist) {
                return false;
            }

            out_closestPoint = closest;
            out_distance = d;
            return true;
        }

        // 8. Face Region
        f32 sum = va + vb + vc;
        f32 denom = 1.0f / sum;
        f32 v = vb * denom;
        f32 w = vc * denom;
        glm::vec3 closest = a + ab * v + ac * w;

        out_closestPoint = closest;
        out_distance = glm::distance(p, closest);
        return true;
    }

}
