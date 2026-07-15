#pragma once
#include <memory>
#include <filesystem>
#include "Types.h"

namespace core::io
{
    // Custom deleter for image memory allocated by stb_image.
    // ImageData owns the pixel buffer and releases it automatically.
    struct ImageDataDeleter
    {
        void operator()(u8* ptr) const;
    };

    // Raw image asset data loaded from disk.
    // This structure represents decoded image pixels and contains no
    // rendering-specific information. It can be consumed by any subsystem
    // that needs image data (for example OpenGL texture creation).
    // Images are stored as 8-bit unsigned RGBA pixel data.
    class ImageData
    {
    public:
        ImageData() = default;

        ImageData(ImageData&&) noexcept = default;
        ImageData& operator=(ImageData&&) noexcept = default;

        ImageData(const ImageData&) = delete;
        ImageData& operator=(const ImageData&) = delete;
    public:
        inline bool Empty() const
        {
            return !pixels;
        }

        inline u32 SizeBytes() const
        {
            return u32(width * height * channels);
        }
    public:
        i32 width    = 0;
        i32 height   = 0;
        i32 channels = 0;
        std::unique_ptr<u8, ImageDataDeleter> pixels;
    };

    // Loads an image file from disk and returns decoded pixel data.
    // Supported formats include PNG, JPG, BMP and TGA.
    // The returned image owns its pixel memory.
    // @param path Path to the image file.
    // @return Decoded image data. Returns an invalid ImageData on failure.
    ImageData LoadImage(const std::filesystem::path& path);

    // Saves image data to disk.
    // The output format is selected from the file extension.
    // Supported formats include PNG, JPG, BMP and TGA.
    // @param path Destination file path.
    // @param image Image data to write.
    void SaveImage(const std::filesystem::path& path, const ImageData& image);
}
