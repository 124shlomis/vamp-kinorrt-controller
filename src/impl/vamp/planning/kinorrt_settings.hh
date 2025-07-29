#pragma once

namespace vamp::planning
{
    struct KinoRRTSettings
    {
        float range = 1.;
        bool dynamic_domain = true;
        float radius = 4.;
        float alpha = 0.0001;
        float min_radius = 1.;
        std::size_t max_iterations = 100000;
        std::size_t max_samples = 100000;
        std::size_t rng_skip_iterations = 1000;
        float min_duration = 0.1f;    // Minimum control duration
        float max_duration = 1.0f;     // Maximum control duration
        float goal_tolerance = 0.1f;   // Goal tolerance
    };
}  // namespace vamp::planning
