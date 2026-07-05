#pragma once
#include <filesystem>
#include <geo/math/RigidTransform.h>
#include <Graphics/BufferObjects.h>
#include <Graphics/Shader.h>

namespace gl
{
    // GPU representation of a point cloud.
    //
    // Owns the OpenGL objects required to render a collection of 3D points.
    // A local rigid transform is maintained internally and applied during
    // rendering. The underlying GPU buffers are immutable after construction.
    //
    // This class is intended purely for visualization and is independent of
    // geo::PointCloud3D or registration algorithms.
    class PointCloudDrawable final
    {
    public:
        PointCloudDrawable() = default;

        // Uploads the given point positions to GPU memory.
        // Each point is rendered as a GL_POINTS primitive.
        PointCloudDrawable(const std::vector<glm::vec3>& points);
    public:
        // Uploads data to gpu.
        void Upload(const std::vector<glm::vec3>& points);
    public:
        // Renders the point cloud.
        // view_projection = Projection * View
        // The internal model transform is applied automatically.
        void Draw(const gl::ShaderProgram& shader,
            const glm::mat4& view_projection,
            const glm::vec3& color,
            geo::f32 pointSize = 3.0f) const;
    public:
        // Applies an additional rigid transform to the drawable.
        // The supplied transform is composed with the existing model transform.
        void Transform(const geo::RigidTransform& transform);
        inline void SetTransform(const geo::RigidTransform& transform) { m_transform = transform; }
        inline const geo::RigidTransform& GetTransform() const { return m_transform; }
    private:
        geo::u32 m_pointCount = 0;
        gl::VertexArray m_vao;
        geo::RigidTransform m_transform;
    };
}
