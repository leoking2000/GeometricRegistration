#pragma once
#include <sstream>
#include "Log.h"
#include "EntryBuilder.h"

#include <cstring>

#define GEO_FILENAME \
    (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : \
    (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__))

#define GEOLOG(msg) { std::ostringstream oss; oss << msg; geo::LogMessage(oss.str()); }

#define GEOLOGFATAL(msg)   if(geo::LogLevel::FATAL   <= geo::GetLogLevel()){ std::ostringstream oss; oss << msg; geo::EntryBuilder(GEO_FILENAME, __LINE__).Fatal(oss.str()); }
#define GEOLOGERROR(msg)   if(geo::LogLevel::ERROR   <= geo::GetLogLevel()){ std::ostringstream oss; oss << msg; geo::EntryBuilder(GEO_FILENAME, __LINE__).Error(oss.str()); }
#define GEOLOGWARN(msg)    if(geo::LogLevel::WARN    <= geo::GetLogLevel()){ std::ostringstream oss; oss << msg; geo::EntryBuilder(GEO_FILENAME, __LINE__).Warn(oss.str()); }
#define GEOLOGINFO(msg)    if(geo::LogLevel::INFO    <= geo::GetLogLevel()){ std::ostringstream oss; oss << msg; geo::EntryBuilder(GEO_FILENAME, __LINE__).Info(oss.str()); }
#define GEOLOGDEBUG(msg)   if(geo::LogLevel::DEBUG   <= geo::GetLogLevel()){ std::ostringstream oss; oss << msg; geo::EntryBuilder(GEO_FILENAME, __LINE__).Debug(oss.str()); }
#define GEOLOGVERBOSE(msg) if(geo::LogLevel::VERBOSE <= geo::GetLogLevel()){ std::ostringstream oss; oss << msg; geo::EntryBuilder(GEO_FILENAME, __LINE__).Verbose(oss.str()); }



