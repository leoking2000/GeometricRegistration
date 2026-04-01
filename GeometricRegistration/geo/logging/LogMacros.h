#pragma once
#include <sstream>
#include <cstring>
#include "Log.h"
#include "EntryBuilder.h"


#define GEO_FILENAME \
    (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : \
    (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__))

#define GEOLOG(msg)        { std::ostringstream oss; oss << msg; geo::LogMessage(oss.str()); }

#define IS_LEVEL_ACTIVE(level) (level <= geo::GetLogLevel() && level != geo::LogLevel::NONE)
#define GEOLOGLEVEL(level, msg) if(IS_LEVEL_ACTIVE(level)){ std::ostringstream oss; oss << msg; geo::EntryBuilder(GEO_FILENAME, __LINE__).SetMessage(oss.str()).SetLogLevel(level); }

#define GEOLOGFATAL(msg)   GEOLOGLEVEL(geo::LogLevel::FATAL,  msg)
#define GEOLOGERROR(msg)   GEOLOGLEVEL(geo::LogLevel::ERROR,  msg)
#define GEOLOGWARN(msg)    GEOLOGLEVEL(geo::LogLevel::WARN,   msg) 
#define GEOLOGINFO(msg)    GEOLOGLEVEL(geo::LogLevel::INFO,   msg) 
#define GEOLOGDEBUG(msg)   GEOLOGLEVEL(geo::LogLevel::DEBUG,  msg)
#define GEOLOGVERBOSE(msg) GEOLOGLEVEL(geo::LogLevel::VERBOSE,msg)
                                       
                                       
                                       
                                       