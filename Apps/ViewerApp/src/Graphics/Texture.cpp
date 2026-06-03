#include <cassert>
#include <glad/glad.h>
#include <geo/utils/logging/LogMacros.h>
#include "Texture.h"

namespace gl
{
    geo::u32 Texture::TYPE[4] = {
        GL_TEXTURE_1D,
        GL_TEXTURE_2D,
        GL_TEXTURE_3D,
        GL_TEXTURE_2D_ARRAY
    };

    Texture::Texture(geo::u32 width, geo::u32 height, TextureFormat format, geo::u8* data)
        :
        Texture(DIM_2D, { width, height, 0 }, format,
            TextureMinFiltering::MIN_NEAREST, TextureMagFiltering::MAG_NEAREST,
            TextureWrapping::CLAMP_TO_EDGE, TextureWrapping::CLAMP_TO_EDGE,
            data)
    {}

    Texture::Texture(
        TextureDimensions dimensions, TexSize size,
        TextureFormat format,
        TextureMinFiltering min_filter, TextureMagFiltering mag_filter,
        TextureWrapping S, TextureWrapping T, geo::u8* data
    )
        :
        m_params(dimensions, size, format, min_filter, mag_filter, S, T)
    {
        glGenTextures(1, &m_id);

        IsTexSizeValid(size);

        SetFiltering(m_params.min_filter, m_params.mag_filter);
        SetWrapping(m_params.wrapping_s, m_params.wrapping_t);
        SetImageData(data, m_params.format);
    }

    Texture::Texture(Texture&& other) noexcept
        :
        m_id(other.m_id),
        m_params(other.m_params)
    {
        other.m_id = 0;
    }

    Texture& Texture::operator=(Texture&& other) noexcept
    {
        glDeleteTextures(1, &m_id);
        m_id = other.m_id;
        m_params = other.m_params;
        other.m_id = 0;
        return *this;
    }

    Texture::~Texture()
    {
        glDeleteTextures(1, &m_id);
    }

    void Texture::Bind(geo::u32 slot) const
    {
        glActiveTexture(GL_TEXTURE0 + slot);
        glBindTexture(TYPE[m_params.dimensions], m_id);
    }

    void Texture::UnBind() const
    {
        glBindTexture(TYPE[m_params.dimensions], 0);
    }

    void Texture::SetFiltering(TextureMinFiltering min_filter, TextureMagFiltering mag_filter)
    {
        m_params.min_filter = min_filter;
        m_params.mag_filter = mag_filter;

        glBindTexture(TYPE[m_params.dimensions], m_id);
        m_minimap = false;

        switch (m_params.min_filter)
        {
        case TextureMinFiltering::MIN_NEAREST:
            glTexParameterf(TYPE[m_params.dimensions], GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            break;
        case TextureMinFiltering::MIN_LINEAR:
            glTexParameterf(TYPE[m_params.dimensions], GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            break;
        case TextureMinFiltering::MIN_NEAREST_MIPMAP_NEAREST:
            glTexParameterf(TYPE[m_params.dimensions], GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
            m_minimap = true;
            break;
        case TextureMinFiltering::MIN_LINEAR_MIPMAP_NEAREST:
            glTexParameterf(TYPE[m_params.dimensions], GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
            m_minimap = true;
            break;
        case TextureMinFiltering::MIN_NEAREST_MIPMAP_LINEAR:
            glTexParameterf(TYPE[m_params.dimensions], GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_LINEAR);
            m_minimap = true;
            break;
        case TextureMinFiltering::MIN_LINEAR_MIPMAP_LINEAR:
            glTexParameterf(TYPE[m_params.dimensions], GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            m_minimap = true;
            break;
        }

        switch (m_params.mag_filter)
        {
        case TextureMagFiltering::MAG_NEAREST:
            glTexParameteri(TYPE[m_params.dimensions], GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            break;
        case TextureMagFiltering::MAG_LINEAR:
            glTexParameteri(TYPE[m_params.dimensions], GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            break;
        }

        glBindTexture(TYPE[m_params.dimensions], 0);
    }

    void Texture::SetWrapping(TextureWrapping S, TextureWrapping T)
    {
        m_params.wrapping_s = S;
        m_params.wrapping_t = T;

        glBindTexture(TYPE[m_params.dimensions], m_id);

        switch (m_params.wrapping_s)
        {
        case TextureWrapping::REPEAT:
            glTexParameteri(TYPE[m_params.dimensions], GL_TEXTURE_WRAP_S, GL_REPEAT);
            break;
        case TextureWrapping::MIRRORED_REPEAT:
            glTexParameteri(TYPE[m_params.dimensions], GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT);
            break;
        case TextureWrapping::CLAMP_TO_EDGE:
            glTexParameteri(TYPE[m_params.dimensions], GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            break;
        case TextureWrapping::CLAMP_TO_BORDER:
            glTexParameteri(TYPE[m_params.dimensions], GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
            break;
        }

        switch (m_params.wrapping_t)
        {
        case TextureWrapping::REPEAT:
            glTexParameteri(TYPE[m_params.dimensions], GL_TEXTURE_WRAP_T, GL_REPEAT);
            break;
        case TextureWrapping::MIRRORED_REPEAT:
            glTexParameteri(TYPE[m_params.dimensions], GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT);
            break;
        case TextureWrapping::CLAMP_TO_EDGE:
            glTexParameteri(TYPE[m_params.dimensions], GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            break;
        case TextureWrapping::CLAMP_TO_BORDER:
            glTexParameteri(TYPE[m_params.dimensions], GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
            break;
        }

        glBindTexture(TYPE[m_params.dimensions], 0);
    }

    void Texture::SetImageData(geo::u8* data, TextureFormat format)
    {
        m_params.format = format;

        glBindTexture(TYPE[m_params.dimensions], m_id);

        GLint internalFormat = 0;
        GLenum color_format = 0;
        GLenum type = 0;

        switch (m_params.format)
        {
        case TextureFormat::RGBA8UB:
            internalFormat = GL_RGBA8;
            color_format = GL_RGBA;
            type = GL_UNSIGNED_BYTE;
            break;
        case TextureFormat::RGBA16F:
            internalFormat = GL_RGBA16F;
            color_format = GL_RGBA;
            type = GL_FLOAT;
            break;
        case TextureFormat::RGBA32F:
            internalFormat = GL_RGBA32F;
            color_format = GL_RGBA;
            type = GL_FLOAT;
            break;
        case TextureFormat::RGBA32UI:
            internalFormat = GL_RGBA32UI;
            color_format = GL_RGBA_INTEGER;
            type = GL_UNSIGNED_INT;
            break;
        case TextureFormat::R32UI:
            internalFormat = GL_R32UI;
            color_format = GL_RED_INTEGER;
            type = GL_UNSIGNED_INT;
            break;
        case TextureFormat::R32F:
            internalFormat = GL_R32F;
            color_format = GL_RED;
            type = GL_FLOAT;
            break;
        case TextureFormat::DEPTH_COMPONENT32F:
            internalFormat = GL_DEPTH_COMPONENT32F;
            color_format = GL_DEPTH_COMPONENT;
            type = GL_FLOAT;
            break;
        }

        switch (m_params.dimensions)
        {
        case DIM_1D:
            glTexImage1D(TYPE[m_params.dimensions], 0,
                internalFormat, m_params.size.x, 0, color_format, type, data);
            break;
        case DIM_2D:
            glTexImage2D(TYPE[m_params.dimensions], 0,
                internalFormat, m_params.size.x, m_params.size.y, 0, color_format, type, data);
            break;
        case DIM_3D:
            glTexImage3D(TYPE[m_params.dimensions], 0,
                internalFormat, m_params.size.x, m_params.size.y, m_params.size.z, 0, color_format, type, data);
            break;
        case DIM_2D_ARRAY:
            glTexImage3D(TYPE[m_params.dimensions], 0,
                internalFormat, m_params.size.x, m_params.size.y, m_params.size.z, 0, color_format, type, data);
            break;
        }

        if (m_minimap)
        {
            glGenerateMipmap(TYPE[m_params.dimensions]);

            //GLfloat anisoSetting = 0.0f; 
            //glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &anisoSetting); 
            //glTexParameterf(TYPE[m_params.dimensions], GL_TEXTURE_MAX_ANISOTROPY_EXT, anisoSetting);
        }

        glBindTexture(TYPE[m_params.dimensions], 0);
    }

    void Texture::Resize(const TexSize& new_size)
    {
        if (IsTexSizeValid(new_size) == false)
        {
            return;
        }

        m_params.size = new_size;
        SetImageData(nullptr, m_params.format);
    }

    bool Texture::IsTexSizeValid(const TexSize& new_size) const
    {
        bool isValid = false;

        if (m_params.dimensions == DIM_1D)
        {
            isValid = new_size.x != 0 && new_size.y == 0 && new_size.z == 0;
        }

        if (m_params.dimensions == DIM_2D)
        {
            isValid = new_size.x != 0 && new_size.y != 0 && new_size.z == 0;
        }

        if (m_params.dimensions == DIM_3D || m_params.dimensions == DIM_2D_ARRAY)
        {
            isValid = new_size.x != 0 && new_size.y != 0 && new_size.z != 0;
        }

        if (!isValid) {
            GEOLOGERROR("Texture size is invalid | size {" << new_size.x << ", " << new_size.y << ", " << new_size.z
            << "dimensions: " << m_params.dimensions + 1);
        }

        return isValid;
    }
}