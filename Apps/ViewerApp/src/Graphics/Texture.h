#pragma once
#include <glm/glm.hpp>
#include <geo/utils/GeoTypes.h>

namespace gl
{
    enum TextureDimensions : geo::u8
    {
        DIM_1D       = 0,
        DIM_2D       = 1,
        DIM_3D       = 2,
        DIM_2D_ARRAY = 3
    };

    enum class TextureFormat : geo::u8
    {
        RGBA8UB,
        RGBA16F,
        RGBA32F,
        RGBA32UI,
        R32UI,
        R32F,
        DEPTH_COMPONENT32F
    };

    enum class TextureWrapping : geo::u8
    {
        REPEAT,
        MIRRORED_REPEAT,
        CLAMP_TO_EDGE,
        CLAMP_TO_BORDER
    };

    enum class TextureMinFiltering : geo::u8
    {
        MIN_NEAREST,
        MIN_LINEAR,
        MIN_NEAREST_MIPMAP_NEAREST,
        MIN_LINEAR_MIPMAP_NEAREST,
        MIN_NEAREST_MIPMAP_LINEAR,
        MIN_LINEAR_MIPMAP_LINEAR
    };

    enum class TextureMagFiltering : geo::u8
    {
        MAG_NEAREST,
        MAG_LINEAR
    };

    class Texture
    {
    public:
        using TexSize = glm::vec<3, geo::u32>;
    public:
        Texture() = default;
        Texture(geo::u32 width, geo::u32 height, TextureFormat format = TextureFormat::RGBA8UB, geo::u8* data = nullptr);

        Texture(TextureDimensions dimensions, TexSize size, TextureFormat format,
            TextureMinFiltering min_filter, TextureMagFiltering mag_filter,
            TextureWrapping S, TextureWrapping T, geo::u8* data
        );

        Texture(const Texture& other) = delete;
        Texture& operator=(const Texture& other) = delete;

        Texture(Texture&& other)  noexcept;
        Texture& operator=(Texture&& other) noexcept;

        ~Texture();
    public:
        inline geo::u32 GetID() const { return m_id; };
        inline TextureDimensions Dimensions() const { return m_params.dimensions; }
        inline TexSize Size() const { return m_params.size; }
    public:
        void Bind(geo::u32 slot = 0) const;
        void UnBind() const;
    public:
        void SetFiltering(TextureMinFiltering min_filter, TextureMagFiltering mag_filter);
        void SetWrapping(TextureWrapping S, TextureWrapping T);
        void SetImageData(geo::u8* data, TextureFormat format);
        void Resize(const TexSize& new_size);
    private:
        bool IsTexSizeValid(const TexSize& new_size) const;
    private:
        struct TextureParameters
        {
            TextureParameters() = default;
            TextureParameters(TextureDimensions dimensions, TexSize size,
                TextureFormat format,
                TextureMinFiltering min_filter, TextureMagFiltering mag_filter,
                TextureWrapping S, TextureWrapping T
            )
                :
                dimensions(dimensions),
                size(size),
                format(format),
                min_filter(min_filter),
                mag_filter(mag_filter),
                wrapping_s(S),
                wrapping_t(T)
            {}

            TextureDimensions dimensions;
            TexSize size;
            TextureFormat format;
            TextureMinFiltering min_filter;
            TextureMagFiltering mag_filter;
            TextureWrapping wrapping_s;
            TextureWrapping wrapping_t;
        };
    private:
        geo::u32 m_id = 0;
        bool m_minimap = false;
        TextureParameters m_params = {};
    private:
        friend class FrameBuffer;
        static geo::u32 TYPE[4];
    };
}