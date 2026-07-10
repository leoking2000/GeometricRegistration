#include <mutex>
#include <sstream>
#include "Log.h"

namespace core
{
    // Global log level controlling filtering of all log output.
    static LogLevel g_logLevel = LogLevel::LOG_INFO;

    // Mutex protecting access to global logging state and output,
    // ensuring thread-safe logging from multiple threads.
    static std::mutex g_logMutex;

    // Sets the global log level in a thread-safe manner.
    // Affects all subsequent logging calls.
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

    // Formats a log entry into a human-readable string.
    //
    // Output format:
    // [LEVEL] message (file:line)
    //
    // Note: msg is assumed to already contain any necessary spacing/prefixing.
    static inline std::string FormatEntry(LogLevel level, std::string_view msg, const char* sourceFile, u32 sourceLine)
    {
        std::ostringstream oss;
        oss << GetLogLevelName(level)
            << msg
            << " (" << (sourceFile ? sourceFile : "<unknown>") << ":" << sourceLine << ")";

        return oss.str();
    }

    // Core logging function (low-level backend).
    void Log(LogLevel level, std::string_view msg, const char* sourceFile, u32 sourceLine, std::ostream& output)
    {
        std::lock_guard<std::mutex> lock(g_logMutex);

        if (!IsLevelActive(level, g_logLevel)) {
            return;
        }

        output << FormatEntry(level, msg, sourceFile, sourceLine) << '\n';
    }
}