#include "EntryBuilder.h"
#include "Log.h"

namespace geo
{
    EntryBuilder::EntryBuilder(const char* file, int line)
        :
        entry{ LogLevel::NONE, "", file, line }
    {
    }

    EntryBuilder::~EntryBuilder()
    {
        LogEntry(entry);
    }

    EntryBuilder& EntryBuilder::SetLogLevel(LogLevel l)
    {
        entry.level = l;
        return *this;
    }

    EntryBuilder& EntryBuilder::SetMessage(std::string n)
    {
        entry.note = std::move(n);
        return *this;
    }

    EntryBuilder& EntryBuilder::Verbose(std::string n)
    {
        entry.level = LogLevel::VERBOSE;
        entry.note = std::move(n);
        return *this;
    }

    EntryBuilder& EntryBuilder::Debug(std::string n)
    {
        entry.level = LogLevel::DEBUG;
        entry.note = std::move(n);
        return *this;
    }

    EntryBuilder& EntryBuilder::Info(std::string n)
    {
        entry.level = LogLevel::INFO;
        entry.note = std::move(n);
        return *this;
    }

    EntryBuilder& EntryBuilder::Warn(std::string n)
    {
        entry.level = LogLevel::WARN;
        entry.note = std::move(n);
        return *this;
    }

    EntryBuilder& EntryBuilder::Error(std::string n)
    {
        entry.level = LogLevel::ERROR;
        entry.note = std::move(n);
        return *this;
    }

    EntryBuilder& EntryBuilder::Fatal(std::string n)
    {
        entry.level = LogLevel::FATAL;
        entry.note = std::move(n);
        return *this;
    }
}
