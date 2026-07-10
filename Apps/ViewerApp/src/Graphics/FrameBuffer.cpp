#include <glad/glad.h>
#include <core/logging/Log.h>
#include "FrameBuffer.h"


namespace gl
{
    FrameBuffer::FrameBuffer(u32 width, u32 height, u32 colorAttachmentCount,
        TextureFormat format, FrameBufferMode fbt, 
        TextureMinFiltering min_filter, TextureMagFiltering mag_filter)
    {
        switch (fbt)
        {
        case gl::FrameBufferMode::ColorAttachment:
            InitColorAttachmentMode(width, height, colorAttachmentCount, min_filter, mag_filter, format);
            break;
        case gl::FrameBufferMode::Layered:
            InitLayered(width, height, colorAttachmentCount, min_filter, mag_filter, format);
            break;
        case gl::FrameBufferMode::Texture3D:
            InitTexture3D(width, height, colorAttachmentCount, min_filter, mag_filter, format);
            break;
        }
    }

    FrameBuffer::FrameBuffer(u32 width, u32 height, u32 depth, u32 colorAttachmentCount, TextureFormat format, TextureMinFiltering min_filter, TextureMagFiltering mag_filter)
    {
        InitColorAttachmentMode3D(width, height, depth, colorAttachmentCount, min_filter, mag_filter, format);
    }

	FrameBuffer::FrameBuffer(FrameBuffer&& other) noexcept
		:
		m_width(other.m_width),
		m_height(other.m_height),
		m_depth(other.m_depth),
		m_id(other.m_id),
		m_depth_texture(std::move(other.m_depth_texture)),
		m_color_attachments(std::move(other.m_color_attachments))
	{
		other.m_width = 0;
		other.m_height = 0;
		other.m_depth = 0;
		other.m_id = 0;
	}

    FrameBuffer& FrameBuffer::operator=(FrameBuffer&& other) noexcept
    {
        m_color_attachments.clear();

        if (m_depth_texture)
        {
            glDeleteTextures(1, &m_depth_texture->m_id);
            m_depth_texture->m_id = 0;
        }

        glDeleteFramebuffers(1, &m_id);

        m_id = other.m_id;
        other.m_id = 0;

        m_width = other.m_width;
        other.m_width = 0;

        m_height = other.m_height;
        other.m_height = 0;

        m_depth = other.m_depth;
        other.m_depth = 0;

        m_depth_texture = std::move(other.m_depth_texture);
        m_color_attachments = std::move(other.m_color_attachments);

        return *this;
    }

    FrameBuffer::~FrameBuffer()
	{
		m_color_attachments.clear();

		if (m_depth_texture)
		{
			glDeleteTextures(1, &m_depth_texture->m_id);
			m_depth_texture->m_id = 0;
		}

		glDeleteFramebuffers(1, &m_id);
	}

    void FrameBuffer::Bind() const
    {
        glBindFramebuffer(GL_FRAMEBUFFER, m_id);

        constexpr static GLenum drawbuffers[32] = {
            GL_COLOR_ATTACHMENT0,
            GL_COLOR_ATTACHMENT1,
            GL_COLOR_ATTACHMENT2,
            GL_COLOR_ATTACHMENT3,
            GL_COLOR_ATTACHMENT4,
            GL_COLOR_ATTACHMENT5,
            GL_COLOR_ATTACHMENT6,
            GL_COLOR_ATTACHMENT7,
            GL_COLOR_ATTACHMENT8,
            GL_COLOR_ATTACHMENT9,
            GL_COLOR_ATTACHMENT10,
            GL_COLOR_ATTACHMENT11,
            GL_COLOR_ATTACHMENT12,
            GL_COLOR_ATTACHMENT13,
            GL_COLOR_ATTACHMENT14,
            GL_COLOR_ATTACHMENT15,
            GL_COLOR_ATTACHMENT16,
            GL_COLOR_ATTACHMENT17,
            GL_COLOR_ATTACHMENT18,
            GL_COLOR_ATTACHMENT19,
            GL_COLOR_ATTACHMENT20,
            GL_COLOR_ATTACHMENT21,
            GL_COLOR_ATTACHMENT22,
            GL_COLOR_ATTACHMENT23,
            GL_COLOR_ATTACHMENT24,
            GL_COLOR_ATTACHMENT25,
            GL_COLOR_ATTACHMENT26,
            GL_COLOR_ATTACHMENT27,
            GL_COLOR_ATTACHMENT28,
            GL_COLOR_ATTACHMENT29,
            GL_COLOR_ATTACHMENT30,
            GL_COLOR_ATTACHMENT31
        };

        glDrawBuffers((GLsizei)m_color_attachments.size(), drawbuffers);
    }

    void FrameBuffer::UnBind() const
    {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    void FrameBuffer::BindColorTexture(u8 index, u32 slot) const
    {
        m_color_attachments[index].Bind(slot);
    }

    void FrameBuffer::BindDepthTexture(u32 slot) const
    {
        if (m_depth_texture) { m_depth_texture->Bind(slot); }
    }

    void FrameBuffer::UnBindColorTexture(u8 index) const
    {
        m_color_attachments[index].UnBind();
    }

    void FrameBuffer::UnBindDepthTexture() const
    {
        if (m_depth_texture) { m_depth_texture->UnBind(); }
    }

    void FrameBuffer::Resize(u32 width, u32 height)
    {
        m_width = width;
        m_height = height;

        if (m_depth_texture)
        {
            m_depth_texture->Resize(Texture::TexSize(m_width, m_height, 0));
        }

        for (Texture& tex : m_color_attachments)
        {
            tex.Resize(Texture::TexSize(m_width, m_height, 0));
        }

        GLenum status = CheckFramebufferStatus(m_id);
    }

    u8 FrameBuffer::NumberOfColorAttachments() const
    {
        return (u8)m_color_attachments.size();
    }

    u32 FrameBuffer::Width() const
    {
        return m_width;
    }

    u32 FrameBuffer::Height() const
    {
        return m_height;
    }

    u32 FrameBuffer::Depth() const
    {
        return m_depth;
    }

    void FrameBuffer::InitColorAttachmentMode(u32 width, u32 height, u32 colorAttachmentCount,
        TextureMinFiltering min_filter, TextureMagFiltering mag_filter, TextureFormat format)
    {
        m_width = width;
        m_height = height;
        m_depth = 0;

        glGenFramebuffers(1, &m_id);
        glBindFramebuffer(GL_FRAMEBUFFER, m_id);

        colorAttachmentCount = CheckColorAttachmentNumber(colorAttachmentCount);

        for (u8 i = 0; i < colorAttachmentCount; i++)
        {
            Texture& tex = m_color_attachments.emplace_back(DIM_2D, Texture::TexSize(m_width, m_height, 0), format,
                min_filter, mag_filter,
                TextureWrapping::CLAMP_TO_EDGE, TextureWrapping::CLAMP_TO_EDGE, (u8*)0u
            );

            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + i, Texture::TYPE[tex.m_params.dimensions], tex.m_id, 0);
        }

        m_depth_texture = Texture(DIM_2D, { m_width, m_height, 0 },
            TextureFormat::DEPTH_COMPONENT32F,
            TextureMinFiltering::MIN_NEAREST, TextureMagFiltering::MAG_NEAREST,
            TextureWrapping::CLAMP_TO_EDGE, TextureWrapping::CLAMP_TO_EDGE, 0);

        glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, m_depth_texture->m_id, 0);


        LOGVERBOSE("FrameBuffer " << m_id <<
            "Created with size " << m_width << "x" << m_height <<
            " with " << colorAttachmentCount << " color attachments"
        );

        if (CheckFramebufferStatus(m_id) != GL_FRAMEBUFFER_COMPLETE)
        {
            LOGERROR("Frame buffer error");
        }
    }

    void FrameBuffer::InitColorAttachmentMode3D(u32 width, u32 height, u32 depth, u32 colorAttachmentCount,
        TextureMinFiltering min_filter, TextureMagFiltering mag_filter, TextureFormat format)
    {
        m_width = width;
        m_height = height;
        m_depth = depth;

        glGenFramebuffers(1, &m_id);
        glBindFramebuffer(GL_FRAMEBUFFER, m_id);

        colorAttachmentCount = CheckColorAttachmentNumber(colorAttachmentCount);

        for (u8 i = 0; i < colorAttachmentCount; i++)
        {
            Texture& tex = m_color_attachments.emplace_back(DIM_3D, Texture::TexSize(m_width, m_height, m_depth), format,
                min_filter, mag_filter,
                TextureWrapping::CLAMP_TO_EDGE, TextureWrapping::CLAMP_TO_EDGE, (u8*)0u
            );

            glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + i, tex.m_id, 0);
        }

        LOGVERBOSE("FrameBuffer " << m_id <<
            "Created with size " << m_width << "x" << m_height << "x" << m_depth <<
            " with " << colorAttachmentCount << " color attachments"
        );

        if (CheckFramebufferStatus(m_id) != GL_FRAMEBUFFER_COMPLETE)
        {
            LOGERROR("Frame buffer error");
        }
    }

    void FrameBuffer::InitLayered(u32 width, u32 height, u32 colorAttachmentCount,
        TextureMinFiltering min_filter, TextureMagFiltering mag_filter, TextureFormat format)
    {
        m_width = width;
        m_height = height;
        m_depth = 0;

        glGenFramebuffers(1, &m_id);
        glBindFramebuffer(GL_FRAMEBUFFER, m_id);

        colorAttachmentCount = CheckColorAttachmentNumber(colorAttachmentCount);

        Texture& tex = m_color_attachments.emplace_back(DIM_2D_ARRAY, Texture::TexSize(m_width, m_height, colorAttachmentCount), format,
            min_filter, mag_filter,
            TextureWrapping::CLAMP_TO_EDGE, TextureWrapping::CLAMP_TO_EDGE, (u8*)0u
        );

        glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, tex.m_id, 0);

        LOGVERBOSE("FrameBuffer " << m_id <<
            "Created with size " << m_width << "x" << m_height <<
            " and " << colorAttachmentCount << " color attachments Layered"
        );

        if (CheckFramebufferStatus(m_id) != GL_FRAMEBUFFER_COMPLETE)
        {
            LOGERROR("Frame buffer error");
        }
    }

    void FrameBuffer::InitTexture3D(u32 width, u32 height, u32 colorAttachmentCount,
        TextureMinFiltering min_filter, TextureMagFiltering mag_filter, TextureFormat format)
    {
        m_width = width;
        m_height = height;
        m_depth = colorAttachmentCount;

        glGenFramebuffers(1, &m_id);
        glBindFramebuffer(GL_FRAMEBUFFER, m_id);

        Texture& tex = m_color_attachments.emplace_back(DIM_3D, Texture::TexSize(m_width, m_height, m_depth), format,
            min_filter, mag_filter,
            TextureWrapping::CLAMP_TO_EDGE, TextureWrapping::CLAMP_TO_EDGE, (u8*)0u
        );

        glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, tex.m_id, 0);

        //LOGDEBUG("FrameBuffer({}): Created with size ({},{},{})", m_id, m_width, m_height, m_depth);
        LOGVERBOSE("FrameBuffer " << m_id <<
            "Created with size " << m_width << "x" << m_height << "x" << m_width
        );

        if (CheckFramebufferStatus(m_id) != GL_FRAMEBUFFER_COMPLETE)
        {
            LOGERROR("Frame buffer error");
        }
    }

    u32 FrameBuffer::CheckColorAttachmentNumber(u32 colorAttachmentCount)
    {
        GLint maxDrawBuf = 0;
        glGetIntegerv(GL_MAX_DRAW_BUFFERS, &maxDrawBuf);

        if (colorAttachmentCount == 0)
        {
            glDrawBuffer(GL_NONE);
            glReadBuffer(GL_NONE);
        }
        else if ((GLint)colorAttachmentCount > maxDrawBuf)
        {
            LOGWARN("FrameBuffer " << m_id << " Attempted to create with " << colorAttachmentCount <<
                " color attachments, " << maxDrawBuf << " will be created insted.");

            colorAttachmentCount = maxDrawBuf;
        }

        return colorAttachmentCount;
    }

    u32 CheckFramebufferStatus(u32 framebuffer_object)
    {
        glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_object);
        u32 status = glCheckFramebufferStatus(GL_FRAMEBUFFER);

        if (status != GL_FRAMEBUFFER_COMPLETE)
        {
            LOGERROR("FrameBuffer " << framebuffer_object << " : glCheckFramebufferStatus: error " << status);
            switch (status)
            {
            case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
                LOGERROR("Incomplete attatchement");
                break;
            case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
                LOGERROR("Incomplete missing attachment");
                break;
            case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
                LOGERROR("Incomplete draw buffer");
                break;
            case GL_FRAMEBUFFER_UNSUPPORTED:
                LOGERROR("Unsupported");
                break;
            case GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS:
                LOGERROR("Incomplete layer targets");
                break;
            default:
                LOGERROR("Default error");
                break;
            }
        }

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        return status;
    }
}
