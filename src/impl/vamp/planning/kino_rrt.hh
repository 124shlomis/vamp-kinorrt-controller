#pragma once

#include <memory>

#include <vamp/collision/environment.hh>
#include <vamp/planning/nn.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/validate.hh>
#include <vamp/planning/kinorrt_settings.hh>
#include <vamp/random/halton.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>

namespace vamp::planning
{
    template <typename Robot, typename RNG, std::size_t rake, std::size_t resolution>
    struct KinoRRT
    {
        using Configuration = typename Robot::Configuration;
        using Control = typename Robot::Control;
        static constexpr auto dimension = Robot::dimension;

        inline static auto solve(
            const Configuration &start,
            const Configuration &goal,
            const collision::Environment<FloatVector<rake>> &environment,
            const KinoRRTSettings &settings) noexcept -> KinoPlanningResult<dimension, Robot::control_dimension>
        {
            return solve(start, std::vector<Configuration>{goal}, environment, settings);
        }

        inline static auto solve(
            const Configuration &start,
            const std::vector<Configuration> &goals,
            const collision::Environment<FloatVector<rake>> &environment,
            const KinoRRTSettings &settings) noexcept -> KinoPlanningResult<dimension, Robot::control_dimension>
        {
            KinoPlanningResult<dimension, Robot::control_dimension> result;
          
            NN<dimension> start_tree;

            constexpr const std::size_t start_index = 0;
            constexpr const std::size_t goal_index = 1;


            auto buffer = std::unique_ptr<float>(
                vamp::utils::vector_alloc<float, FloatVectorAlignment, FloatVectorWidth>(
                    settings.max_samples * Configuration::num_scalars_rounded));

            const auto buffer_index = [&buffer](std::size_t index) -> float *
            { return buffer.get() + index * Configuration::num_scalars_rounded; };

            std::vector<std::size_t> parents(settings.max_samples);
            std::vector<float> radii(settings.max_samples);
            // Store control inputs and durations for each node
            std::vector<FloatVector<Robot::control_dimension>> node_controls(settings.max_samples);
            std::vector<float> node_durations(settings.max_samples);

            auto start_time = std::chrono::steady_clock::now();


            // tree
            auto *tree_a = &start_tree;

            RNG rng(settings.rng_skip_iterations);
            vamp::rng::Halton<Robot::control_dimension> control_rng;
            vamp::rng::Distribution duration_rng;
            std::size_t iter = 0;
            std::size_t free_index = start_index + 1;

            // add start to tree
            start.to_array(buffer_index(start_index));
            start_tree.insert(NNNode<dimension>{start_index, {buffer_index(start_index)}});
            parents[start_index] = start_index;
            radii[start_index] = std::numeric_limits<float>::max();

            bool done = false;
            while (iter++ < settings.max_iterations and free_index < settings.max_samples)
            {

                auto temp = rng.next();
                Robot::scale_configuration(temp);

                typename Robot::ConfigurationBuffer temp_array;
                temp.to_array(temp_array.data());

                const auto nearest = tree_a->nearest(NNFloatArray<dimension>{temp_array.data()}); // TODO: nearest should take angular distace
                if (not nearest)
                {
                    continue;
                }

                const auto &[nearest_node, nearest_distance] = *nearest;
                const auto nearest_radius = radii[nearest_node.index];

                if (settings.dynamic_domain and nearest_radius < nearest_distance)
                {
                    continue;
                }

                const auto nearest_configuration = nearest_node.as_vector();


                float duration = duration_rng.uniform_real(settings.min_duration, settings.max_duration);
                auto control_input = control_rng.next();
                Robot::scale_control(control_input);
                //Control control_input = Control(temp_ctrl);  
                //Robot::scale_control(temp);
                Configuration new_configuration;  

                if (validate_control<Robot, rake, resolution>(
                        nearest_configuration,
                        control_input, 
                        duration,
                        environment,
                        new_configuration))
                {
                    float *new_configuration_index = buffer_index(free_index);
                    //auto new_configuration = nearest_configuration + extension_vector;
                    new_configuration.to_array(new_configuration_index);
                    tree_a->insert(NNNode<dimension>{free_index, {new_configuration_index}});

                    parents[free_index] = nearest_node.index;
                    radii[free_index] = std::numeric_limits<float>::max();
                    
                    // Store the control input and duration used to reach this node
                    node_controls[free_index] = control_input;
                    node_durations[free_index] = duration;

                    free_index++;

                    if (settings.dynamic_domain and nearest_radius != std::numeric_limits<float>::max())
                    {
                        radii[nearest_node.index] *= (1 + settings.alpha);
                    }

                    for (const auto &goal : goals)
                    {
                        auto dist = Robot::calculate_distance(new_configuration, goal);
                        if (dist < settings.goal_tolerance)
                        {
                            done = true;
                            break;
                        }
                    }
                    if (done)
                    {
                        auto current = free_index - 1;
                        result.path.emplace_back(buffer_index(current));
                        while (parents[current] != current)
                        {
                            auto parent = parents[current];
                            result.path.emplace_back(buffer_index(parent));
                            result.controls.emplace_back(node_controls[current]);
                            result.durations.emplace_back(node_durations[current]);
                            result.cost += result.path[result.path.size() - 1].distance(
                                result.path[result.path.size() - 2]);
                            current = parent;
                        }

                        std::reverse(result.path.begin(), result.path.end());
                        std::reverse(result.controls.begin(), result.controls.end());
                        std::reverse(result.durations.begin(), result.durations.end());
                        break;
                    }
                }
                else if (settings.dynamic_domain)
                {
                    if (nearest_radius == std::numeric_limits<float>::max())
                    {
                        radii[nearest_node.index] = settings.radius;
                    }
                    else
                    {
                        radii[nearest_node.index] =
                            std::max(radii[nearest_node.index] * (1.F - settings.alpha), settings.min_radius);
                    }
                }
                
            }

            result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
            result.iterations = iter;
            result.size.emplace_back(start_tree.size());
            return result;
        }
    };
}  // namespace vamp::planning

