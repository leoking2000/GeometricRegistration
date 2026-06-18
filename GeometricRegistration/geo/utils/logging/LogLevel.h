#pragma once
#include <string_view>
#include <geo/utils/GeoTypes.h>

namespace geo
{
    // Log severity levels used to filter and categorize runtime messages.
    // Higher numeric value = more verbose output.
    enum class LogLevel : u8
    {
        LOG_NONE    = 0u,  // No logging output
        LOG_FATAL   = 1u,  // Critical errors that likely terminate execution
        LOG_ERROR   = 2u,  // Errors that indicate failure of an operation
        LOG_WARN    = 3u,  // Potential issues or non-fatal problems
        LOG_INFO    = 4u,  // General runtime information
        LOG_DEBUG   = 5u,  // Debugging details for development
        LOG_VERBOSE = 6u   // Highly detailed trace-level output
    };

    // Returns true if a log message of a given level should be emitted
    // given the current global log level.
    //
    // Behavior:
    // - LOG_NONE disables all output
    // - Messages are allowed if level <= current verbosity level
    constexpr inline bool IsLevelActive(LogLevel level, LogLevel current)
    {
        if (current == LogLevel::LOG_NONE) return false;
        return static_cast<u8>(level) <= static_cast<u8>(current);
    }

    // Returns a fixed-width human-readable label for each log level.
    // These strings are formatted with padding to support aligned console output.
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
            default:                    return "[Unknown]  ";
        }
    }
}
