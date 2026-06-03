#pragma once
#include <vector>
#include <optional>
#include "Texture.h"


namespace gl
{
	enum class FrameBufferMode
	{
		ColorAttachment,
		Layered,
		Texture3D
	};

	class FrameBuffer
	{
	public:
		// for 2D Texture
		FrameBuffer(geo::u32 width, geo::u32 height, geo::u32 colorAttachmentCount = 1,
			TextureFormat format = TextureFormat::RGBA32F, FrameBufferMode fbt = FrameBufferMode::ColorAttachment,
			TextureMinFiltering min_filter = TextureMinFiltering::MIN_NEAREST,
			TextureMagFiltering mag_filter = TextureMagFiltering::MAG_NEAREST);

		// for 3D Texture
		FrameBuffer(geo::u32 width, geo::u32 height, geo::u32 depth, geo::u32 colorAttachmentCount = 1,
			TextureFormat format = TextureFormat::RGBA32F,
			TextureMinFiltering min_filter = TextureMinFiltering::MIN_NEAREST,
			TextureMagFiltering mag_filter = TextureMagFiltering::MAG_NEAREST);

		FrameBuffer(const FrameBuffer&) = delete;
		FrameBuffer& operator=(const FrameBuffer&) = delete;

		FrameBuffer(FrameBuffer&& other) noexcept;
		FrameBuffer& operator=(FrameBuffer&& other) noexcept;

		~FrameBuffer();
	public:
		void Bind() const;
		void UnBind() const;

		void BindColorTexture(geo::u8 index, geo::u32 slot) const;
		void BindDepthTexture(geo::u32 slot) const;

		void UnBindColorTexture(geo::u8 index) const;
		void UnBindDepthTexture() const;

		void Resize(geo::u32 width, geo::u32 height);

		geo::u8 NumberOfColorAttachments() const;

		geo::u32 Width() const;
		geo::u32 Height() const;
		geo::u32 Depth() const;
	private:
		void InitColorAttachmentMode(geo::u32 width, geo::u32 height, geo::u32 colorAttachmentCount,
			TextureMinFiltering min_filter, TextureMagFiltering mag_filter, TextureFormat format);
		void InitColorAttachmentMode3D(geo::u32 width, geo::u32 height, geo::u32 depth, geo::u32 colorAttachmentCount,
			TextureMinFiltering min_filter, TextureMagFiltering mag_filter, TextureFormat format);

		void InitLayered(geo::u32 width, geo::u32 height, geo::u32 colorAttachmentCount,
			TextureMinFiltering min_filter, TextureMagFiltering mag_filter, TextureFormat format);
		void InitTexture3D(geo::u32 width, geo::u32 height, geo::u32 colorAttachmentCount,
			TextureMinFiltering min_filter, TextureMagFiltering mag_filter, TextureFormat format);

		geo::u32 CheckColorAttachmentNumber(geo::u32 colorAttachmentCount);
	private:
		geo::u32 m_id     = 0;
		geo::u32 m_width  = 0;
		geo::u32 m_height = 0;
		geo::u32 m_depth  = 0;

		std::optional<Texture> m_depth_texture;
		std::vector<Texture> m_color_attachments;
	};

	geo::u32 CheckFramebufferStatus(geo::u32 framebuffer_object);
}
