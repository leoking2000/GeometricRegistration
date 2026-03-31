#include "Log.h"
#include <mutex>
#include <iostream>
#include <sstream>

namespace geo
{
    static LogLevel g_logLevel = LogLevel::INFO;
    static std::mutex g_logMutex;

    void SetLogLevel(LogLevel level)
    {
        g_logLevel = level;
    }

    LogLevel GetLogLevel()
    {
        return g_logLevel;
    }

    void LogMessage(std::string_view message)
    {
        std::lock_guard<std::mutex> lock(g_logMutex);
        std::cout << message << std::endl;
    }

    void LogEntry(const Entry& e)
    {
        std::ostringstream builder;
        builder << GetLogLevelName(e.level);
        builder << e.note;
        builder << "  (" << e.sourceFile << ":" << e.sourceLine << ")";

        std::string msg = builder.str();
        LogMessage(msg);
    }
}