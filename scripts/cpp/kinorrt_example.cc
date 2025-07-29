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

#include <fstream>

using Robot = vamp::robots::Ackermann;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using RNG = vamp::rng::Halton<Robot::dimension>;
using KinoRRT = vamp::planning::KinoRRT<Robot, RNG, rake, Robot::resolution>;

// Start and goal configurations
static constexpr Robot::ConfigurationArray start = {0., 0., 0., 0.}; // x, y, z, theta
static constexpr Robot::ConfigurationArray goal = {2.35, 1., 0., 0.};

// Spheres for the cage problem - (x, y, z) center coordinates with fixed, common radius defined below
static const std::vector<std::array<float, 3>> problem = {
    {0.55, 0., 0.}
};

// Radius for obstacle spheres
static constexpr float radius = 0.2;

void log_data(const vamp::planning::KinoPlanningResult<Robot::dimension, Robot::control_dimension> &result)
{
    // Logs the planning result to CSV files.
    // Input: result - KinoPlanningResult containing the planned path, controls, and durations.
    // Output files:
    //   - path.csv: Contains the robot's path (x, y, theta) for each configuration.
    //   - obstacles.csv: Contains obstacle centers and their radius.
    //   - controls.csv: Contains control inputs (velocity, steering) and their durations.

    // Output configurations to CSV
    std::ofstream path_file("path.csv");
    std::ofstream obs_file("obstacles.csv");
    std::ofstream control_file("controls.csv");

    if (!path_file.is_open() || !obs_file.is_open() || !control_file.is_open())
    {
        std::cerr << "Failed to open output CSV files." << std::endl;
        return;
    }

    path_file << std::fixed << std::setprecision(5);
    for (const auto &config : result.path)
    {
        const auto &array = config.to_array(); // x, y, z, theta
        path_file << array[0] << "," << array[1] << "," << array[3] << "\n";  // x, y, theta
    }

    // Write obstacle centers and radius
    for (const auto &obs : problem)
    {
        obs_file << obs[0] << "," << obs[1] << "," << radius << "\n";  // x, y, r
    }
    
    // controls and durations
    control_file << std::fixed << std::setprecision(5);
    for (std::size_t i = 0; i < result.controls.size(); ++i)
    {
        const auto &control = result.controls[i].to_array();
        control_file << control[0] << "," << control[1] << "," << result.durations[i] << "\n";  // v, steer, duration
    }

    std::cout << "Path, Control and obstacle data written to CSV.\n";
}

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

    auto result =
        KinoRRT::solve(Robot::Configuration(start), Robot::Configuration(goal), env_v, kino_rrt_settings);

    // If successful
    if (result.path.size() > 0)
    {
        // Simplify path with default settings
        /*vamp::planning::SimplifySettings simplify_settings;
        auto simplify_result = vamp::planning::simplify<Robot, rake, Robot::resolution>(
            result.path, env_v, simplify_settings, rng);*/

        // Output configurations of simplified path
        std::cout << std::fixed << std::setprecision(3);
        for (const auto &config : result.path)
        {
            const auto &array = config.to_array();
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                std::cout << array[i] << ", ";
            }

            std::cout << std::endl;
        }
        log_data(result);
    }

    return 0;
}
