#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>

#include <vamp/collision/factory.hh>
#include <vamp/planning/validate.hh>
#include <vamp/planning/kino_rrt.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/robots/ackermann.hh>
#include <vamp/random/halton.hh>

using Robot = vamp::robots::Ackermann;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using RNG = vamp::rng::Halton<Robot::dimension>;
using KinoRRT = vamp::planning::KinoRRT<Robot, RNG, rake, Robot::resolution>;

// Start and goal configurations
static constexpr Robot::ConfigurationArray start = {0., 0., 0.};
static constexpr Robot::ConfigurationArray goal = {2.35, 1., 0.};

// Spheres for the cage problem - (x, y, z) center coordinates with fixed, common radius defined below
static const std::vector<std::array<float, 3>> problem = {
    {0.55, 0, 0.25},
    {0.35, 0.35, 0.25},
    {0, 0.55, 0.25},
    {-0.55, 0, 0.25},
    {-0.35, -0.35, 0.25},
    {0, -0.55, 0.25},
    {0.35, -0.35, 0.25},
    {0.35, 0.35, 0.8},
    {0, 0.55, 0.8},
    {-0.35, 0.35, 0.8},
    {-0.55, 0, 0.8},
    {-0.35, -0.35, 0.8},
    {0, -0.55, 0.8},
    {0.35, -0.35, 0.8},
};

// Radius for obstacle spheres
static constexpr float radius = 0.2;

auto main(int, char **) -> int
{
    // Build sphere cage environment
    EnvironmentInput environment;
    for (const auto &sphere : problem)
    {
        environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, radius));
    }

    environment.sort();
    auto env_v = EnvironmentVector(environment);

    // Create RNG for planning
    auto rng = std::make_shared<vamp::rng::Halton<Robot::dimension>>();

    // Setup KinoRRT and plan
    vamp::planning::KinoRRTSettings kino_rrt_settings;
    kino_rrt_settings.range = 1.0;

    auto result =
        KinoRRT::solve(Robot::Configuration(start), Robot::Configuration(goal), env_v, kino_rrt_settings);

    // If successful
    if (result.path.size() > 0)
    {
        // Simplify path with default settings
        vamp::planning::SimplifySettings simplify_settings;
        auto simplify_result = vamp::planning::simplify<Robot, rake, Robot::resolution>(
            result.path, env_v, simplify_settings, rng);

        // Output configurations of simplified path
        std::cout << std::fixed << std::setprecision(3);
        for (const auto &config : simplify_result.path)
        {
            const auto &array = config.to_array();
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                std::cout << array[i] << ", ";
            }

            std::cout << std::endl;
        }
    }

    return 0;
}
