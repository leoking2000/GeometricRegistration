#pragma once
#include "EntryBuilder.h"


#define LEOLOGFATAL(msg)   geo::EntryBuilder{__FILE__, __LINE__}.Fatal(msg)
#define LEOLOGERROR(msg)   geo::EntryBuilder{__FILE__, __LINE__}.Error(msg)
#define LEOLOGWARN(msg)    geo::EntryBuilder{__FILE__, __LINE__}.Warn(msg)

#if !PRODUCTION_BUILD
	#define LEOLOGINFO(msg)    geo::EntryBuilder{__FILE__, __LINE__}.Info(msg)
	#define LEOLOGDEBUG(msg)   geo::EntryBuilder{__FILE__, __LINE__}.Debug(msg)
	#define LEOLOGVERBOSE(msg) geo::EntryBuilder{__FILE__, __LINE__}.Verbose(msg)
#else
	#define LEOLOGWARN(msg)
	#define LEOLOGINFO(msg)
	#define LEOLOGDEBUG(msg)
	#define LEOLOGVERBOSE(msg)
#endif




