#include "Log.h"
#include <mutex>
#include <sstream>

namespace geo
{
    static LogLevel g_logLevel = LogLevel::INFO;
    static std::mutex g_logMutex;

    void SetLogLevel(LogLevel level)
    {
        std::lock_guard<std::mutex> lock(g_logMutex);
        g_logLevel = level;
    }

    LogLevel GetLogLevel()
    {
        std::lock_guard<std::mutex> lock(g_logMutex);
        return g_logLevel;
    }

    static inline std::string FormatEntry(LogLevel level, std::string_view msg, const char* sourceFile, u32 sourceLine)
    {
        std::ostringstream oss;
        oss << GetLogLevelName(level)
            << msg
            << " (" << (sourceFile ? sourceFile : "<unknown>") << ":" << sourceLine << ")";

        return oss.str();
    }

    void Log(LogLevel level, std::string_view msg, const char* sourceFile, u32 sourceLine, std::ostream& output)
    {
        std::lock_guard<std::mutex> lock(g_logMutex);

        if (!IsLevelActive(level, g_logLevel)) {
            return;
        }

        output << FormatEntry(level, msg, sourceFile, sourceLine) << '\n';
    }
}