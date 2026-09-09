#pragma once
#ifndef PRODUCTION_BUILD
#define PRODUCTION_BUILD 0
#endif

#if !PRODUCTION_BUILD // if not in a PRODUCTION BUILD

#include <cassert>
#define CORE_ASSERT(condition) assert(condition)
#define CORE_ASSERT_MSG(condition, msg) assert((condition) && (msg))

#else

#define CORE_ASSERT(condition)
#define CORE_ASSERT_MSG(condition, msg)

#endif // !PRODUCTION_BUILD
