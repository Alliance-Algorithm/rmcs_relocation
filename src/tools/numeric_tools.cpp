#include "tools/numeric_tools.hpp"

#include <algorithm>
#include <cmath>

namespace rmcs::location::tools {

auto sanitize_non_negative(double value, double fallback) -> double {
    if (std::isfinite(value) && value >= 0.0)
        return value;
    return fallback;
}

auto sanitize_positive_int(int value, int fallback) -> int {
    return std::max(1, value > 0 ? value : fallback);
}

auto sanitize_step(double step_deg, double fallback_deg) -> double {
    if (std::isfinite(step_deg) && step_deg > 1e-6)
        return step_deg;
    return fallback_deg;
}

auto sanitize_iterations(int value, int fallback) -> int {
    return std::max(1, value > 0 ? value : fallback);
}

} // namespace rmcs::location::tools
