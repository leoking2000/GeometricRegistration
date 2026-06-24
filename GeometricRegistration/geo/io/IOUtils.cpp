#include <fstream>
#include <geo/utils/logging/LogMacros.h>
#include "IOUtils.h"

namespace geo::io
{
    // ============================================================
    // Utilities
    // ============================================================

    std::string ReadFile(const std::filesystem::path& filePath)
    {
        std::string filename = filePath.string();
        std::ifstream input_file(filename.c_str(), std::ios::binary);

        if (!input_file.is_open())
        {
            GEOLOGERROR("Failed to open file at " << filename.c_str());
            return std::string();
        }

        return std::string((std::istreambuf_iterator<char>(input_file)), std::istreambuf_iterator<char>());
    }

    bool FileExists(const std::filesystem::path& filePath)
    {
        return std::filesystem::exists(filePath);
    }

    std::string GetFileName(const std::filesystem::path& filePath)
    {
        return filePath.filename().string();
    }

    std::filesystem::path GetParentFolder(const std::filesystem::path& filePath)
    {
        return filePath.parent_path();
    }
}
