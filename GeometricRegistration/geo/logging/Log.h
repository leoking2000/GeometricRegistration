#pragma once
#include "EntryBuilder.h"


#define LOGFATAL(msg)   geo::EntryBuilder{__FILE__, __LINE__}.Fatal(msg)
#define LOGERROR(msg)   geo::EntryBuilder{__FILE__, __LINE__}.Error(msg)
#define LOGWARN(msg)    geo::EntryBuilder{__FILE__, __LINE__}.Warn(msg)

#if !PRODUCTION_BUILD
	#define LOGINFO(msg)    geo::EntryBuilder{__FILE__, __LINE__}.Info(msg)
	#define LOGDEBUG(msg)   geo::EntryBuilder{__FILE__, __LINE__}.Debug(msg)
	#define LOGVERBOSE(msg) geo::EntryBuilder{__FILE__, __LINE__}.Verbose(msg)
#else
	#define LOGWARN(msg)
	#define LOGINFO(msg)
	#define LOGDEBUG(msg)
	#define LOGVERBOSE(msg)
#endif
