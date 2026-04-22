#pragma once
#include <ostream>
#include "LogLevel.h"

namespace geo
{
    // Sets the global log level (affects all logging)
    void SetLogLevel(LogLevel level);
    // Returns the current global log level
    LogLevel GetLogLevel();

    // Low-level logging backend. Prefer using macros/wrappers.
    void Log(LogLevel level, std::string_view msg, const char* sourceFile, u32 sourceLine, std::ostream& output);
}
