#pragma once

#include <chrono>

namespace rmcs::location::tools {

auto sanitize_non_negative(double value, double fallback) -> double;
auto sanitize_positive_int(int value, int fallback) -> int;
auto sanitize_step(double step_deg, double fallback_deg) -> double;
auto sanitize_iterations(int value, int fallback) -> int;

inline auto as_steady_duration(double seconds) -> std::chrono::steady_clock::duration {
    return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(seconds));
}

template <typename Clock, typename Duration>
inline auto elapsed_sec(
    const std::chrono::time_point<Clock, Duration>& since,
    const std::chrono::time_point<Clock, Duration>& now) -> double {
    return std::chrono::duration<double>(now - since).count();
}

} // namespace rmcs::location::tools
