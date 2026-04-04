#pragma once
#include <chrono>
#include <string>
#include "GeoTypes.h"
#include "logging/LogMacros.h"

namespace geo
{
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    // Compute a - b in milliseconds
    f64 TimeDifferenceMs(TimePoint end, TimePoint start);

    struct TimingStat
    {
    public:
        f64 totalMs = 0.0;
        u32 count   = 0;
    public:
        void AddSample(f64 sample_ms);
        f64 AverageMs() const;
        bool Empty() const;
        std::string ToString() const;
    };

    class ScopedTimer
    {
    public:
        explicit ScopedTimer(TimingStat* stat);
        explicit ScopedTimer(const std::string& name, TimingStat* stat, LogLevel level = NONE);

        ScopedTimer(const ScopedTimer&) = delete;
        ScopedTimer& operator=(const ScopedTimer&) = delete;

        ~ScopedTimer();
    public:
    private:
        std::string m_name  = "";
        TimePoint m_start   = Clock::now();
        TimingStat* m_stat  = nullptr;
        LogLevel m_logLevel = NONE;
    };

    class Stopwatch
    {
    public:
        Stopwatch();
    public:
        void Start();
        f64 StopMs();
        f64 RestartMs();
        f64 ElapsedMs() const;
        bool IsRunning() const;
    private:
        TimePoint m_start = Clock::now();
        bool m_running = false;
    };
}
