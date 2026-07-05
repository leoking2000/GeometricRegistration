#pragma once
#include <geo/GeoTypes.h>
#include <vector>
#include <glm/glm.hpp>

namespace gl
{
	enum class BufferUsage : geo::u8
	{
		Static,
		Dynamic,
		Stream
	};

	enum class ElementType : geo::u8
	{
		FLOAT1, FLOAT1_N,
		FLOAT2, FLOAT2_N,
		FLOAT3, FLOAT3_N,
		FLOAT4, FLOAT4_N,
		UCHAR3, UCHAR3_N,
		UCHAR4, UCHAR4_N,
		MAT4
	};

	struct VertexAttributeDesc
	{
		geo::u32 type;
		geo::u32 count;
		bool normalized;
		geo::u32 size_bytes;
	};

	VertexAttributeDesc GetAttributeDesc(ElementType t);

	class VertexBuffer
	{
	public:
		VertexBuffer() = default;
		VertexBuffer(const void* data, geo::u32 size, BufferUsage usage = BufferUsage::Static);

		VertexBuffer(const VertexBuffer& other) = delete;
		VertexBuffer& operator=(const VertexBuffer&) = delete;

		VertexBuffer(VertexBuffer&& other) noexcept;
		VertexBuffer& operator=(VertexBuffer&& other) noexcept;

		~VertexBuffer();
	public:
		void Bind() const;
		void UnBind() const;
	private:
		geo::u32 m_id = 0;
	};

	class IndexBuffer
	{
	public:
		IndexBuffer() = default;
		IndexBuffer(const geo::u32* data, geo::u32 count, BufferUsage usage = BufferUsage::Static);

		IndexBuffer(const IndexBuffer& other) = delete;
		IndexBuffer& operator=(const IndexBuffer& other) = delete;

		IndexBuffer(IndexBuffer&& other) noexcept;
		IndexBuffer& operator=(IndexBuffer&& other) noexcept;

		~IndexBuffer();
	public:
		void Bind() const;
		void UnBind() const;

		inline geo::u32 GetCount() const { return m_count; }
	private:
		geo::u32 m_id    = 0;
		geo::u32 m_count = 0;
	};

	/*
	* Represents the attribute layout of a VBO
	*/
	template<geo::u32 ELEMENTS_COUNT>
	class Layout
	{
	public:
		Layout(ElementType* element_arr)
		{
			assert(element_arr != nullptr && "Element type array is null");
			m_stride = 0;
			for (geo::u32 i = 0; i < ELEMENTS_COUNT; i++)
			{
				if (element_arr != nullptr)
				{
					m_arr[i] = element_arr[i];
					m_stride += GetAttributeDesc(element_arr[i]).size_bytes;
				}
			}
		}

		inline ElementType operator[](int i) const { return m_arr[i]; }
		inline geo::u32 GetStride() const { return m_stride; }
		inline geo::u32 GetCount() const { return ELEMENTS_COUNT; }
	private:
		ElementType m_arr[ELEMENTS_COUNT] = {};
		geo::u32 m_stride = 0;
	};

	class VertexArray
	{
	public:
		VertexArray();

		VertexArray(const VertexArray& other) = delete;
		VertexArray& operator=(const VertexArray&) = delete;

		VertexArray(VertexArray&& other) noexcept;
		VertexArray& operator=(VertexArray&& other) noexcept;

		~VertexArray();
	public:
		void Bind() const;
		void UnBind() const;
		inline geo::u32 ID() const { return m_id; }
	public:
		template<geo::u32 ELEMENTS_COUNT>
		void AddBuffer(VertexBuffer&& vb, const Layout<ELEMENTS_COUNT>& layout, geo::u32 start = 0, bool per_instance = false)
		{
			Bind();
			vb.Bind();
			geo::u32 offset = 0;
			for (geo::u32 i = start; i < start + ELEMENTS_COUNT; i++)
			{
				AddAttrib(i, layout[i - start], layout.GetStride(), offset, per_instance);
			}
			UnBind();
			vb.UnBind();
			m_buffers.emplace_back(std::move(vb));
		}

		void SetIndexBuffer(IndexBuffer&& ib)
		{
			Bind();
			ib.Bind();
			m_indexBuffer = std::move(ib);
			UnBind();
		}
	private:
		void AddAttrib(geo::u32 i, ElementType element_type, geo::u32 stride, geo::u32& offset, bool per_instance);

		geo::u32 m_id;
		std::vector<VertexBuffer> m_buffers;
		IndexBuffer m_indexBuffer; // default empty
	};
}