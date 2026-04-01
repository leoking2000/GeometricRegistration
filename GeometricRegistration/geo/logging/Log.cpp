#include "Log.h"
#include <mutex>
#include <iostream>
#include <sstream>
#include <chrono>
#include <iomanip>

namespace geo
{
    static LogLevel g_logLevel = LogLevel::INFO;
    static std::mutex g_logMutex;

    static inline std::string GetTimeString()
    {
        using namespace std::chrono;

        auto now = system_clock::now();
        std::time_t t = system_clock::to_time_t(now);

        std::tm tm{};
#ifdef _WIN32
        localtime_s(&tm, &t);
#else
        localtime_r(&t, &tm);
#endif

        std::ostringstream oss;
        oss << std::put_time(&tm, "%H:%M:%S");
        return oss.str();
    }

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
        builder << "[" << GetTimeString() << "]";
        builder << GetLogLevelName(e.level);
        builder << e.note;
        builder << "  (" << e.sourceFile << ":" << e.sourceLine << ")";

        std::string msg = builder.str();
        LogMessage(msg);
    }
}