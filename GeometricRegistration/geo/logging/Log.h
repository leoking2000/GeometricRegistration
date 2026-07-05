#pragma once
#include <ostream>
#include "LogLevel.h"

namespace geo
{
    // Sets the global log level in a thread-safe manner.
    // Affects all subsequent logging calls.
    void SetLogLevel(LogLevel level);

    // Retrieves the current global logging verbosity level.
    LogLevel GetLogLevel();

    // Low-level logging backend function.
    // This is the core logging entry point used by higher-level macros/wrappers.
    // It is not intended to be used directly in most application code.
    // Responsibilities:
    // - Filters messages based on global log level
    // - Formats log entry with source information
    // - Writes output to provided stream
    // - Ensures thread safety via mutex
    // Output format:
    // [LEVEL] message (file:line)
    //
    // @param level       Severity of the log message
    // @param msg         Preformatted log message text
    // @param sourceFile  File from which the log was emitted
    // @param sourceLine  Line number in the source file
    // @param output      Output stream to write the log message to
    void Log(LogLevel level, std::string_view msg, const char* sourceFile, u32 sourceLine, std::ostream& output);
}
