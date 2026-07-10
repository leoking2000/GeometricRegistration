#pragma once
#include <string>
#include "BufferObjects.h"

namespace gl
{
	enum class DefaultMesh
	{
		CUBE,
		SPHERE,
		QUAD,
		SCREEN_FILLED_QUAD
	};

    struct Vertex
    {
        glm::vec3 pos;
        glm::vec2 texCord;
        glm::vec3 normal;

        bool operator==(const Vertex& rhs) const
        {
            return pos == rhs.pos &&
                texCord == rhs.texCord &&
                normal == rhs.normal;
        }
    };

    class Mesh
    {
    public:
        // Constructors
        Mesh() = default;
        Mesh(DefaultMesh shape);
        Mesh(DefaultMesh shape, const glm::mat4* model_arr, u32 count);

        Mesh(VertexArray& va, IndexBuffer& ib, u32 layout_size);
        Mesh(VertexArray& va, IndexBuffer& ib, u32 layout_size, const glm::mat4* model_arr, u32 count);

        Mesh(const Mesh&) = delete;
        Mesh& operator=(const Mesh&) = delete;

        Mesh(Mesh&& other) noexcept;
        Mesh& operator=(Mesh&& other) noexcept;

        ~Mesh() = default;
    public:
        // Drawing
        void Draw() const;
    public:
        // Factory functions
        static Mesh GenerateMesh(DefaultMesh shape);
        static Mesh GenerateCube();
        static Mesh GenerateSphere(u32 precision = 48);
        static Mesh GenerateQuad(u32 repeat = 1);
        static Mesh GenerateScreenFilledQuad();
    public:
        // Instancing
        bool HasInstanceArray() const;
        void MakeInstancedArray(const glm::mat4* model_arr, u32 count);
    private:
        u32 m_count = 0;
        u32 m_layout_size = 0;
        VertexArray m_vertexArray;
        IndexBuffer m_indexBuffer;
    };
}
