#pragma once
#include <chrono>
#include <string>
#include "GeoTypes.h"
#include "logging/LogLevel.h"

namespace geo
{
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    // TimeDifferenceMs(end, start) returns end - start in milliseconds.
    f64 TimeDifferenceMs(TimePoint end, TimePoint start);

    struct TimingStat
    {
    public:
        f64 totalMs = 0.0;
        u32 count   = 0;
    public:
        // Negative samples are clamped to 0 to keep aggregate timing stats well-formed.
        void AddSample(f64 sample_ms);
        f64 AverageMs() const;
        bool Empty() const;
        std::string ToString() const;
    };

    // ScopedTimer accumulates into TimingStat if provided and optionally logs on scope exit.
    // Logging is emitted only when both a non-empty name and a non-NONE log level are provided.
    class ScopedTimer
    {
    public:
        explicit ScopedTimer(TimingStat* stat);
        explicit ScopedTimer(const std::string& name, TimingStat* stat, LogLevel level = LogLevel::NONE);

        ScopedTimer(const ScopedTimer&) = delete;
        ScopedTimer& operator=(const ScopedTimer&) = delete;

        ~ScopedTimer();
    private:
        std::string m_name  = "";
        TimePoint m_start   = Clock::now();
        TimingStat* m_stat  = nullptr;
        LogLevel m_logLevel = LogLevel::NONE;
    };

    // Stopwatch is edge-triggered: ElapsedMs() and StopMs() return 0 when not running.
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
