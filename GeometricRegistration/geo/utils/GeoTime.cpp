#include <geo/Logging/LogMacros.h>
#include <algorithm>
#include <limits>
#include <sstream>
#include <iomanip>
#include "GeoTime.h"

namespace geo
{
    // Converts a time interval (end - start) into seconds.
    // Uses floating-point duration for sub-second precision.
    f64 TimeDifferenceSec(TimePoint end, TimePoint start)
    {
        return std::chrono::duration<f64>(end - start).count();
    }

    // Converts a time interval (end - start) into milliseconds.
    // Uses floating-point duration to preserve fractional milliseconds.
    f64 TimeDifferenceMs(TimePoint end, TimePoint start)
    {
        return std::chrono::duration<f64, std::milli>(end - start).count();
    }


    // Adds a timing sample (in milliseconds) to the accumulator.
    // Negative values are clamped to zero for robustness.
    void TimingStat::AddSample(f64 sample_ms)
    {
        if (sample_ms < 0.0){
            sample_ms = 0.0;
        }

        m_totalMs += sample_ms;
        m_count += 1;
    }

    // Computes the average of all recorded timing samples (in ms).
    // Returns 0 if no samples exist to avoid division by zero.
    f64 TimingStat::AverageMs() const
    {
        if (m_count == 0){
            return 0.0;
        }

        return m_totalMs / (f64)m_count;
    }

    // Produces a human-readable summary string of timing statistics.
    // Format:
    // count=<N> total=<X>ms avg=<Y>ms
    std::string TimingStat::ToString() const
    {
        std::ostringstream oss;

        if (IsEmpty())
        {
            oss << "count=0 total=0ms avg=0ms";
            return oss.str();
        }

        oss << std::fixed << std::setprecision(3) 
            << "count=" << m_count
            << " total=" << m_totalMs << "ms"
            << " avg=" << AverageMs() << "ms";

        return oss.str();
    }

    // ScopedTimer constructor (non-logging variant).
    // Starts timing immediately; optionally accumulates into TimingStat.
    ScopedTimer::ScopedTimer(TimingStat* stat)
        :
        m_start(Clock::now()),
        m_stat(stat)
    {
    }

    // ScopedTimer constructor with optional logging.
    // Logging is controlled by:
    // - non-empty name
    // - log level != LOG_NONE
    ScopedTimer::ScopedTimer(const std::string& name, TimingStat* stat, LogLevel level)
        : m_name(name)
        , m_start(Clock::now())
        , m_stat(stat)
        , m_logLevel(level)
    {
    }

    // Destructor stops timing automatically (RAII behavior).
    // Responsibilities:
    // - compute elapsed time
    // - accumulate into TimingStat if provided
    // - optionally log timing information
    ScopedTimer::~ScopedTimer()
    {
        const TimePoint end = Clock::now();
        const f64 elapsedMs = TimeDifferenceMs(end, m_start);

        if (m_stat != nullptr)
        {
            m_stat->AddSample(elapsedMs);
        }

        if (!m_name.empty() && m_logLevel != LogLevel::LOG_NONE)
        {
            GEOLOGLEVEL(m_logLevel, m_name << " took " << elapsedMs << " ms");
        }
    }

    // Initializes stopwatch in a non-running state with a reference start time.
    Stopwatch::Stopwatch()
        : m_start(Clock::now())
        , m_running(false)
    {
    }

    // Stops timing and returns elapsed milliseconds.
    // If stopwatch is not running, returns 0 safely.
    void Stopwatch::Start()
    {
        m_start = Clock::now();
        m_running = true;
    }

    // Stops timing and returns elapsed milliseconds.
    // If stopwatch is not running, returns 0 safely.
    f64 Stopwatch::StopMs()
    {
        if (!m_running)
        {
            return 0.0;
        }

        m_running = false;

        const TimePoint end = Clock::now();
        const f64 elapsedMs = TimeDifferenceMs(end, m_start);
        return elapsedMs;
    }

    // Stops current timing interval, returns elapsed time,
    // and immediately restarts from the current time.
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

    // Returns elapsed time in milliseconds without stopping the timer.
    // If not running, returns 0.
    f64 Stopwatch::ElapsedMs() const
    {
        if (!m_running)
        {
            return 0.0;
        }

        return TimeDifferenceMs(Clock::now(), m_start);
    }

    // Returns whether the stopwatch is actively running.
    bool Stopwatch::IsRunning() const
    {
        return m_running;
    }
}
