#pragma once
#include <ostream>
#include <sstream>
#include <cstring>
#include <iostream>
#include "LogLevel.h"

namespace core
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

// Extracts only the filename portion from a full file path.
//
// Handles both Windows ('\\') and Unix ('/') path separators.
// Falls back to __FILE__ if no separator is found.
#define FILENAME \
    (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : \
    (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__))

// Checks whether a given log level is currently enabled by the global logger.
#define IS_LEVEL_ACTIVE(level) (core::IsLevelActive((level), core::GetLogLevel()))

// Core logging macro.
//
// Behavior:
// - Checks if the log level is enabled
// - Streams message into an ostringstream (supports operator << chaining)
// - Sends formatted log entry to geo::Log
#define LOGLEVEL(level, msg) if(IS_LEVEL_ACTIVE(level)){ std::ostringstream oss; oss << msg; core::Log(level, oss.str(), FILENAME, __LINE__, std::cout); }

// Convenience macros for each log severity level.
// These reduce verbosity and standardize usage across the codebase.

#define LOGFATAL(msg)   LOGLEVEL(core::LogLevel::LOG_FATAL,   msg)
#define LOGERROR(msg)   LOGLEVEL(core::LogLevel::LOG_ERROR,   msg)
#define LOGWARN(msg)    LOGLEVEL(core::LogLevel::LOG_WARN,    msg) 
#define LOGINFO(msg)    LOGLEVEL(core::LogLevel::LOG_INFO,    msg) 
#define LOGDEBUG(msg)   LOGLEVEL(core::LogLevel::LOG_DEBUG,   msg)
#define LOGVERBOSE(msg) LOGLEVEL(core::LogLevel::LOG_VERBOSE, msg)
