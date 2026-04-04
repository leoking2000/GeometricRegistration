#include <algorithm>
#include <limits>
#include <sstream>
#include "GeoTime.h"

namespace geo
{
    f64 TimeDifferenceMs(TimePoint end, TimePoint start)
    {
        return std::chrono::duration<f64, std::milli>(end - start).count();
    }

    void TimingStat::AddSample(f64 sample_ms)
    {
        if (sample_ms < 0.0){
            sample_ms = 0.0;
        }

        totalMs += sample_ms;
        count += 1;
    }

    f64 TimingStat::AverageMs() const
    {
        if (count == 0){
            return 0.0;
        }

        return totalMs / (f64)count;
    }

    bool TimingStat::Empty() const
    {
        return count == 0;
    }

    std::string TimingStat::ToString() const
    {
        std::ostringstream oss;

        if (Empty())
        {
            oss << "count=0 total=0ms avg=0ms";
            return oss.str();
        }

        oss << "count=" << count
            << " total=" << totalMs << "ms"
            << " avg=" << AverageMs() << "ms";

        return oss.str();
    }

    ScopedTimer::ScopedTimer(TimingStat* stat)
        : m_name("")
        , m_start(Clock::now())
        , m_stat(stat)
        , m_logLevel(NONE)
    {
    }

    ScopedTimer::ScopedTimer(const std::string& name, TimingStat* stat, LogLevel level)
        : m_name(name)
        , m_start(Clock::now())
        , m_stat(stat)
        , m_logLevel(level)
    {
    }

    ScopedTimer::~ScopedTimer()
    {
        const TimePoint end = Clock::now();
        const f64 elapsedMs = TimeDifferenceMs(end, m_start);

        if (m_stat != nullptr)
        {
            m_stat->AddSample(elapsedMs);
        }

        if (!m_name.empty())
        {
            GEOLOGLEVEL(m_logLevel, m_name << " took " << elapsedMs << " ms");
        }
    }

    Stopwatch::Stopwatch()
        : m_start(Clock::now())
        , m_running(false)
    {
    }

    void Stopwatch::Start()
    {
        m_start = Clock::now();
        m_running = true;
    }

    f64 Stopwatch::StopMs()
    {
        if (!m_running)
        {
            return 0.0;
        }

        const TimePoint end = Clock::now();
        const f64 elapsedMs = TimeDifferenceMs(end, m_start);
        m_running = false;
        return elapsedMs;
    }

    f64 Stopwatch::RestartMs()
    {
        const TimePoint now = Clock::now();

        if (!m_running)
        {
            m_start = now;
            m_running = true;
            return 0.0;
        }

        const f64 elapsedMs = TimeDifferenceMs(now, m_start);
        m_start = now;
        m_running = true;
        return elapsedMs;
    }

    f64 Stopwatch::ElapsedMs() const
    {
        if (!m_running)
        {
            return 0.0;
        }

        return TimeDifferenceMs(Clock::now(), m_start);
    }

    bool Stopwatch::IsRunning() const
    {
        return m_running;
    }
}