#pragma once
#include "Entry.h"

namespace geo
{
    class EntryBuilder
    {
    public:
        EntryBuilder(const char* file, int line);
        ~EntryBuilder();

        EntryBuilder& SetLogLevel(LogLevel l);
        EntryBuilder& SetMessage(std::string n);

        EntryBuilder& Verbose(std::string n);
        EntryBuilder& Debug(std::string n);
        EntryBuilder& Info(std::string n);
        EntryBuilder& Warn(std::string n);
        EntryBuilder& Error(std::string n);
        EntryBuilder& Fatal(std::string n);
    private:
        Entry entry;
    };
}
