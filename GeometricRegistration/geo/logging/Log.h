#pragma once
#include "Entry.h"

namespace geo
{
    void SetLogLevel(LogLevel level);
    LogLevel GetLogLevel();

    void LogMessage(std::string_view message);
    void LogEntry(const Entry& e);
}
