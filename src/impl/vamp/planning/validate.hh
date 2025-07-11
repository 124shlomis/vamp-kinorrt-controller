#pragma once

#include <cstdint>

#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>

namespace vamp::planning
{
    template <std::size_t n, std::size_t... I>
    inline constexpr auto generate_percents(std::index_sequence<I...>) -> std::array<float, n>
    {
        return {(static_cast<void>(I), static_cast<float>(I + 1) / static_cast<float>(n))...};
    }

    template <std::size_t n>
    struct Percents
    {
        inline static constexpr auto percents = generate_percents<n>(std::make_index_sequence<n>());
    };

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_vector(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &vector,
        float distance,
        const collision::Environment<FloatVector<rake>> &environment) -> bool
    {
        // TODO: Fix use of reinterpret_cast in pack() so that this can be constexpr
        const auto percents = FloatVector<rake>(Percents<rake>::percents);

        typename Robot::template ConfigurationBlock<rake> block;

        // HACK: broadcast() implicitly assumes that the rake is exactly VectorWidth
        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            block[i] = start.broadcast(i) + (vector.broadcast(i) * percents);
        }

        const std::size_t n = std::max(std::ceil(distance / static_cast<float>(rake) * resolution), 1.F);

        bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, block) :
                                                 Robot::template fkcc<rake>(environment, block);
        if (not valid or n == 1)
        {
            return valid;
        }

        const auto backstep = vector / (rake * n);
        for (auto i = 1U; i < n; ++i)
        {
            for (auto j = 0U; j < Robot::dimension; ++j)
            {
                block[j] = block[j] - backstep.broadcast(j);
            }

            if (not Robot::template fkcc<rake>(environment, block))
            {
                return false;
            }
        }

        return true;
    }

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_motion(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &goal,
        const collision::Environment<FloatVector<rake>> &environment) -> bool
    {
        auto vector = goal - start;
        return validate_vector<Robot, rake, resolution>(start, vector, vector.l2_norm(), environment);
    }
    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_control(
        const typename Robot::Configuration &start,
        const typename Robot::Control &control,
        float duration,
        const collision::Environment<FloatVector<rake>> &environment,
        const typename Robot::Configuration &result) -> bool
    {
        // TODO: implment validation of control
        // This is a placeholder implementation that always returns true.
        // In a real implementation, you would check if the control leads to a valid state
        // transition without collisions or violations of robot constraints.
        (void)start;  // Suppress unused variable warning
        (void)control;  // Suppress unused variable warning
        (void)duration;  // Suppress unused variable warning
        (void)environment;  // Suppress unused variable warning
        (void)result;  // Suppress unused variable warning
        return true;
    }
}  // namespace vamp::planning
