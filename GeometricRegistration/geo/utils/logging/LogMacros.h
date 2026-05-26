#pragma once
#include <sstream>
#include <cstring>
#include <iostream>
#include "Log.h"

// Extracts only the filename portion from a full file path.
//
// Handles both Windows ('\\') and Unix ('/') path separators.
// Falls back to __FILE__ if no separator is found.
#define GEO_FILENAME \
    (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : \
    (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__))

// Checks whether a given log level is currently enabled by the global logger.
#define IS_LEVEL_ACTIVE(level) (geo::IsLevelActive((level), geo::GetLogLevel()))

// Core logging macro.
//
// Behavior:
// - Checks if the log level is enabled
// - Streams message into an ostringstream (supports operator << chaining)
// - Sends formatted log entry to geo::Log
#define GEOLOGLEVEL(level, msg) if(IS_LEVEL_ACTIVE(level)){ std::ostringstream oss; oss << msg; geo::Log(level, oss.str(), GEO_FILENAME, __LINE__, std::cout); }

// Convenience macros for each log severity level.
// These reduce verbosity and standardize usage across the codebase.

#define GEOLOGFATAL(msg)   GEOLOGLEVEL(geo::LogLevel::LOG_FATAL,   msg)
#define GEOLOGERROR(msg)   GEOLOGLEVEL(geo::LogLevel::LOG_ERROR,   msg)
#define GEOLOGWARN(msg)    GEOLOGLEVEL(geo::LogLevel::LOG_WARN,    msg) 
#define GEOLOGINFO(msg)    GEOLOGLEVEL(geo::LogLevel::LOG_INFO,    msg) 
#define GEOLOGDEBUG(msg)   GEOLOGLEVEL(geo::LogLevel::LOG_DEBUG,   msg)
#define GEOLOGVERBOSE(msg) GEOLOGLEVEL(geo::LogLevel::LOG_VERBOSE, msg)
