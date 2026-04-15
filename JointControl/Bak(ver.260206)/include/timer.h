#ifndef TIMER_H
#define TIMER_H

#include <chrono>
#include <string>
#include <unordered_map>
#include <stdexcept>

class Timer {
public: // Make TimerData public for static access
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = std::chrono::time_point<Clock>;
    using Duration = std::chrono::duration<double, std::milli>; // Duration in milliseconds

    struct TimerData {
        TimePoint startTime;
        double lastDurationMs = 0.0;
        double totalDurationMs = 0.0;
        bool running = false;
    };

private:
    // Static map to hold timer data
    static std::unordered_map<std::string, TimerData> timers_;

public:
    // Starts or restarts the timer with the given name
    static void start(const std::string& name) {
        timers_[name].startTime = Clock::now();
        timers_[name].running = true;
    }

    // Stops the timer with the given name and records the duration
    static void end(const std::string& name) {
        auto it = timers_.find(name);
        if (it != timers_.end() && it->second.running) {
            TimePoint endTime = Clock::now();
            Duration elapsed = endTime - it->second.startTime;
            it->second.lastDurationMs = elapsed.count();
            it->second.totalDurationMs += it->second.lastDurationMs;
            it->second.running = false;
        }
        // Optional: Handle error if timer wasn't started or doesn't exist
        // else { throw std::runtime_error("Timer '" + name + "' was not started or does not exist."); }
    }

    // Gets the duration of the last start-end interval for the given timer name
    // Returns 0.0 if the timer hasn't completed a cycle or doesn't exist
    static double getElapsedTime(const std::string& name) /* removed const */ {
        auto it = timers_.find(name);
        if (it != timers_.end()) {
            return it->second.lastDurationMs;
        }
        return 0.0;
    }

    // Gets the total accumulated time for the given timer name across all start-end intervals
    // Returns 0.0 if the timer doesn't exist or has never run
    static double getTotalTime(const std::string& name) /* removed const */ {
        auto it = timers_.find(name);
        if (it != timers_.end()) {
            return it->second.totalDurationMs;
        }
        return 0.0;
    }

    // Resets a specific timer
    static void reset(const std::string& name) {
         auto it = timers_.find(name);
         if (it != timers_.end()) {
            it->second.lastDurationMs = 0.0;
            it->second.totalDurationMs = 0.0;
            it->second.running = false;
            it->second.startTime = TimePoint{};
         }
    }

    // Resets all timers
    static void resetAll() {
        timers_.clear(); // Simplest way to reset all
    }
};

inline std::unordered_map<std::string, Timer::TimerData> Timer::timers_;

#endif // TIMER_H