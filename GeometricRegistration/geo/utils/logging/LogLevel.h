#pragma once
#include <string_view>
#include <geo/utils/GeoTypes.h>

namespace geo
{
    enum class LogLevel : u8
    {
        NONE    = 0u,
        FATAL   = 1u,
        ERROR   = 2u,
        WARN    = 3u,
        INFO    = 4u,
        DEBUG   = 5u,
        VERBOSE = 6u
    };

    constexpr inline bool IsLevelActive(LogLevel level, LogLevel current)
    {
        if (current == LogLevel::NONE) return false;
        return static_cast<u8>(level) <= static_cast<u8>(current);
    }

    // Fixed-width labels for aligned console output
    constexpr inline std::string_view GetLogLevelName(LogLevel level)
    {
        switch (level)
        {
            case LogLevel::VERBOSE: return "[Verbose]  ";
            case LogLevel::DEBUG:   return "[Debug]    ";
            case LogLevel::INFO:    return "[Info]     ";
            case LogLevel::WARN:    return "[Warning]  ";
            case LogLevel::ERROR:   return "[Error]    ";
            case LogLevel::FATAL:   return "[Fatal]    ";
            case LogLevel::NONE:    return "[None]     ";
            default:                return "[Unknown]  ";
        }
    }
}
