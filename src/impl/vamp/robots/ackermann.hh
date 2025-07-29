#pragma once

#include <vamp/robots/ackermann/fk.hh>
#include <vamp/vector.hh>

namespace vamp::robots
{
    struct Ackermann
    {
        static constexpr auto name = "ackermann";
        static constexpr auto dimension = 4; // x,y,z,theta
        static constexpr auto control_dimension = 2; // v, steer
        static constexpr auto duration_dimension = 1;
        static constexpr auto resolution = 32;
        static constexpr auto n_spheres = ackermann::n_spheres;
        static constexpr auto space_measure = ackermann::space_measure;

        using Configuration = FloatVector<dimension>;
        using Control = FloatVector<control_dimension>;
        using ConfigurationArray = std::array<FloatT, dimension>;

        struct alignas(FloatVectorAlignment) ConfigurationBuffer
          : std::array<float, Configuration::num_scalars_rounded>
        {
        };

        template <std::size_t rake>
        using ConfigurationBlock = ackermann::ConfigurationBlock<rake>;

        template <std::size_t rake>
        using Spheres = ackermann::Spheres<rake>;

        static constexpr auto scale_configuration = ackermann::scale_configuration;
        static constexpr auto descale_configuration = ackermann::descale_configuration;

        static constexpr auto scale_control = ackermann::scale_control;
        static constexpr auto descale_control = ackermann::descale_control;

        template <std::size_t rake>
        static constexpr auto scale_configuration_block = ackermann::scale_configuration_block<rake>;

        template <std::size_t rake>
        static constexpr auto descale_configuration_block = ackermann::descale_configuration_block<rake>;

        template <std::size_t rake>
        static constexpr auto fkcc = ackermann::interleaved_sphere_fk<rake>;
		
		// Currently not implemented
        template <std::size_t rake>
        static constexpr auto fkcc_attach = ackermann::interleaved_sphere_fk<rake>;

        template <std::size_t rake>
        static constexpr auto sphere_fk = ackermann::sphere_fk<rake>;

        static constexpr auto fp = ackermann::fp_euler;  // { fp_euler, fp_rk4 }
        // Currently not implemented
        static constexpr auto eefk = ackermann::eefk;
    };
}  // namespace vamp::robots
