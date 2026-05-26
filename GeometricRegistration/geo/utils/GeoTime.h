#pragma once
#include <chrono>
#include <string>
#include "GeoTypes.h"
#include "logging/LogLevel.h"

namespace geo
{
    // Alias for the clock type used across the library.
    // steady_clock is used because it is monotonic and not affected by system time changes.
    using Clock = std::chrono::steady_clock;

    // Canonical time point type used for all timing utilities.
    using TimePoint = Clock::time_point;

    // Computes elapsed time in seconds: (end - start).
    // Returns a floating-point value for sub-second precision.
    f64 TimeDifferenceSec(TimePoint end, TimePoint start);

    // Computes elapsed time in milliseconds: (end - start).
    // Returns a floating-point value for sub-millisecond precision.
    f64 TimeDifferenceMs(TimePoint end, TimePoint start);

    // Utility struct for accumulating multiple timing samples.
    // Useful for benchmarking repeated operations and computing statistics.
    struct TimingStat
    {
    public:
        // Adds a new timing sample in milliseconds to the accumulator.
        void AddSample(f64 sample_ms);
        // Returns the arithmetic mean of all collected samples (ms).
        f64 AverageMs() const;
    public:
        // Total accumulated time in milliseconds across all samples.
        f64 TotalMs() const { return m_totalMs; }
        // Number of samples recorded.
        u32 Count() const { return m_count; }
        // Returns true if no samples have been recorded.
        bool IsEmpty() const { m_count == 0; };
        // Produces a human-readable summary string of timing statistics.
        // Format:
        // count=<N> total=<X>ms avg=<Y>ms
        std::string ToString() const;
    private:
        f64 m_totalMs = 0.0; // Sum of all recorded timing samples (ms).
        u32 m_count = 0; // Number of samples recorded.
    };

    // RAII-style scoped timer.
    //
    // Measures elapsed time between construction and destruction.
    // Optionally:
    // - accumulates result into a TimingStat
    // - logs timing information on scope exit
    class ScopedTimer
    {
    public:
        // Creates a timer without logging.
        // If stat is provided, results will be accumulated into it.
        explicit ScopedTimer(TimingStat* stat);
        // Creates a named timer with optional logging.
        // Logging occurs only if:
        // - name is non-empty
        // - log level is not LOG_NONE
        explicit ScopedTimer(const std::string& name, TimingStat* stat, LogLevel level = LogLevel::LOG_NONE);

        ScopedTimer(const ScopedTimer&) = delete;
        ScopedTimer& operator=(const ScopedTimer&) = delete;

        ~ScopedTimer(); // Stops timer and optionally logs / records timing.
    private:
        std::string m_name  = "";  // Optional label used for logging output.
        TimePoint m_start   = Clock::now();  // Start timestamp captured at construction.
        TimingStat* m_stat  = nullptr; // Optional external accumulator for timing statistics.
        LogLevel m_logLevel = LogLevel::LOG_NONE; // Log severity level controlling whether output is emitted.
    };

    // Simple stopwatch with explicit start/stop control.
    //
    // Unlike ScopedTimer, timing is manually controlled and can be restarted.
    // Edge-triggered behavior:
    // - If not running, queries return 0.
    class Stopwatch
    {
    public:
        Stopwatch();  // Initializes stopwatch (not started).
    public:
        // Starts timing from current moment.
        void Start();
        // Stops timing and returns elapsed time in milliseconds.
        f64 StopMs();
        // Stops current timing interval, returns elapsed time,
        // and restarts immediately.
        f64 RestartMs();
        // Returns elapsed time in milliseconds without stopping.
        // Returns 0 if stopwatch is not running.
        f64 ElapsedMs() const;
        // Returns whether the stopwatch is currently running.
        bool IsRunning() const;
    private:
        TimePoint m_start = Clock::now(); // Timestamp of last start/restart.
        bool m_running = false; // Indicates whether stopwatch is actively measuring time.
    };
}
