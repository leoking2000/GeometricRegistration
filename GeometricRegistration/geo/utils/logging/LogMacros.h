#pragma once
#include <sstream>
#include <cstring>
#include <iostream>
#include "Log.h"


#define GEO_FILENAME \
    (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : \
    (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__))

#define IS_LEVEL_ACTIVE(level) (geo::IsLevelActive((level), geo::GetLogLevel()))
#define GEOLOGLEVEL(level, msg) if(IS_LEVEL_ACTIVE(level)){ std::ostringstream oss; oss << msg; geo::Log(level, oss.str(), GEO_FILENAME, __LINE__, std::cout); }

#define GEOLOGFATAL(msg)   GEOLOGLEVEL(geo::LogLevel::LOG_FATAL,   msg)
#define GEOLOGERROR(msg)   GEOLOGLEVEL(geo::LogLevel::LOG_ERROR,   msg)
#define GEOLOGWARN(msg)    GEOLOGLEVEL(geo::LogLevel::LOG_WARN,    msg) 
#define GEOLOGINFO(msg)    GEOLOGLEVEL(geo::LogLevel::LOG_INFO,    msg) 
#define GEOLOGDEBUG(msg)   GEOLOGLEVEL(geo::LogLevel::LOG_DEBUG,   msg)
#define GEOLOGVERBOSE(msg) GEOLOGLEVEL(geo::LogLevel::LOG_VERBOSE, msg)
