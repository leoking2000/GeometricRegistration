#pragma once
#include <string>
#include "LogLevel.h"

namespace geo
{
    struct Entry
    {
        LogLevel level = LogLevel::NONE;
        std::string note = "";
        const char* sourceFile = "";
        int sourceLine = 0;
    };
}
