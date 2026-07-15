#include <stb/stb_image.h>
#include <stb/stb_image_write.h> // there is a stb.cpp file with the #defines
#include "logging/Log.h"
#include "IOImage.h"

namespace core::io
{
    void ImageDataDeleter::operator()(u8* ptr) const
    {
        if (ptr != nullptr) {
            stbi_image_free(ptr);
        }
    }

    ImageData LoadImage(const std::filesystem::path& path)
    {
        ImageData image_data;

        stbi_set_flip_vertically_on_load(1);

        int width = 0;
        int height = 0;
        int channels = 0;

        std::string filepath = path.string();
        u8* raw = stbi_load(filepath.c_str(), &width, &height, &channels, 4);

        if (raw == nullptr)
        {
            LOGERROR("Failed to read image data From: " << filepath);

            // Fallback 2x2 magenta checker
            const u8 fallback[16] = {
                255,   0, 255, 255,   0,   0,   0, 255,
                  0,   0,   0, 255, 255,   0, 255, 255
            };

            u8* fallbackHeap = static_cast<u8*>(std::malloc(16));
            if (!fallbackHeap)
            {
                LOGERROR("Fallback allocation failed for image: " << filepath);
                return image_data; // empty image
            }

            std::memcpy(fallbackHeap, fallback, 16);

            image_data.width    = 2;
            image_data.height   = 2;
            image_data.channels = 4;
            image_data.pixels.reset(fallbackHeap);

            return image_data;
        }

        image_data.width    = width;
        image_data.height   = height;
        image_data.channels = 4;

        image_data.pixels.reset(raw);

        return image_data;
    }

    void SaveImage(const std::filesystem::path& path, const ImageData& image)
    {
        if (image.Empty())
        {
            LOGERROR("Cannot save invalid image.");
            return;
        }

        const std::string ext = path.extension().string();
        const std::string filepath = path.string();
        bool success = false;

        if (ext == ".png")
        {
            success = stbi_write_png(
                filepath.c_str(),
                image.width,
                image.height,
                image.channels,
                image.pixels.get(),
                image.width * image.channels
            ) != 0;
        }
        else if (ext == ".jpg" || ext == ".jpeg")
        {
            success = stbi_write_jpg(
                filepath.c_str(),
                image.width,
                image.height,
                image.channels,
                image.pixels.get(),
                95 // quality
            ) != 0;
        }
        else if (ext == ".bmp")
        {
            success = stbi_write_bmp(
                filepath.c_str(),
                image.width,
                image.height,
                image.channels,
                image.pixels.get()
            ) != 0;
        }
        else if (ext == ".tga")
        {
            success = stbi_write_tga(
                filepath.c_str(),
                image.width,
                image.height,
                image.channels,
                image.pixels.get()
            ) != 0;
        }
        else
        {
            LOGERROR("Unsupported image format: " << ext);
            return;
        }

        if (!success)
        {
            LOGERROR("Failed to save image: " << path);
            return;
        }

        LOGINFO("Saved image: \"" << path.string() << "\"");
    }

}