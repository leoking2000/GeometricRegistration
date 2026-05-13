#pragma once
#include <string_view>
#include <geo/utils/GeoTypes.h>

namespace geo
{
    enum class LogLevel : u8
    {
        LOG_NONE    = 0u,
        LOG_FATAL   = 1u,
        LOG_ERROR   = 2u,
        LOG_WARN    = 3u,
        LOG_INFO    = 4u,
        LOG_DEBUG   = 5u,
        LOG_VERBOSE = 6u
    };

    constexpr inline bool IsLevelActive(LogLevel level, LogLevel current)
    {
        if (current == LogLevel::LOG_NONE) return false;
        return static_cast<u8>(level) <= static_cast<u8>(current);
    }

    // Fixed-width labels for aligned console output
    constexpr inline std::string_view GetLogLevelName(LogLevel level)
    {
        switch (level)
        {
            case LogLevel::LOG_VERBOSE: return "[Verbose]  ";
            case LogLevel::LOG_DEBUG:   return "[Debug]    ";
            case LogLevel::LOG_INFO:    return "[Info]     ";
            case LogLevel::LOG_WARN:    return "[Warning]  ";
            case LogLevel::LOG_ERROR:   return "[Error]    ";
            case LogLevel::LOG_FATAL:   return "[Fatal]    ";
            case LogLevel::LOG_NONE:    return "[None]     ";
            default:                return "[Unknown]  ";
        }
    }
}
