#pragma once

#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/collision/validity.hh>

#include <cmath> // added by Yaniv

// NOLINTBEGIN(*-magic-numbers)
namespace vamp::robots::ackermann
{
    using Configuration = FloatVector<4>;   // {x, y, z, θ̇}
    using Control = FloatVector<2>;		 // {v, steer}

    template <std::size_t rake>
    using ConfigurationBlock = FloatVector<rake, 4>;

    // Pad and align vectors for easy loading.
    alignas(FloatVectorAlignment) static std::array<float, FloatVectorWidth> lows{-10, -10,0, 0};
    alignas(FloatVectorAlignment) static std::array<float, FloatVectorWidth> highs{10, 10,5, 6.283185}; // 2*pi
    static float radius = 0.2;

    alignas(FloatVectorAlignment) static std::array<float, FloatVectorWidth> control_lows{0.1, -0.261799};
    alignas(FloatVectorAlignment) static std::array<float, FloatVectorWidth> control_highs{5, 0.261799};  // 15 degrees

    inline void set_radius(float new_radius) noexcept
    {
        radius = new_radius;
    }

    inline void set_lows(std::array<float, 4> new_lows) noexcept
    {
        std::copy_n(new_lows.cbegin(), 4, lows.begin());
    }

    inline void set_highs(std::array<float, 4> new_highs) noexcept
    {
        std::copy_n(new_highs.cbegin(), 4, highs.begin());
    }

    inline void scale_configuration(Configuration &q) noexcept
    {
        Configuration clow(lows.data());
        Configuration chigh(highs.data());

        q = q * (chigh - clow) + clow;
    }

    inline void descale_configuration(Configuration &q) noexcept
    {
        Configuration clow(lows.data());
        Configuration chigh(highs.data());

        q = (q - clow) / (chigh - clow);
    }

    inline void scale_control(Control &q) noexcept
    { //[0,1] -> [low, high]
        Control clow(control_lows.data());
        Control chigh(control_highs.data());

        q = q * (chigh - clow) + clow;
    }

    inline void descale_control(Control &q) noexcept
    { //[low, high] -> [0,1]
        Control clow(control_lows.data());
        Control chigh(control_highs.data());

        q = (q - clow) / (chigh - clow);
    }

    template <std::size_t rake>
    inline void scale_configuration_block(ConfigurationBlock<rake> &q) noexcept
    {
        q[0] = lows[0] + (q[0] * (highs[0] - lows[0]));
        q[1] = lows[1] + (q[1] * (highs[1] - lows[1]));
        q[2] = lows[2] + (q[2] * (highs[2] - lows[2]));
        q[3] = lows[3] + (q[3] * (highs[3] - lows[3]));
    }

    template <std::size_t rake>
    inline void descale_configuration_block(ConfigurationBlock<rake> &q) noexcept
    {
        q[0] = (q[0] - lows[0]) / (highs[0] - lows[0]);
        q[1] = (q[1] - lows[1]) / (highs[1] - lows[1]);
        q[2] = (q[2] - lows[2]) / (highs[2] - lows[2]);
        q[3] = (q[3] - lows[3]) / (highs[3] - lows[3]);
    }

    inline static auto space_measure() noexcept -> float
    {
        Configuration clow(lows.data());
        Configuration chigh(highs.data());
        return (chigh - clow).l2_norm();
    }

    constexpr auto n_spheres = 1;

    template <std::size_t rake>
    struct Spheres
    {
        FloatVector<rake, 1> x;
        FloatVector<rake, 1> y;
        FloatVector<rake, 1> z;
        FloatVector<rake, 1> r;
    };

    template <std::size_t rake>
    inline void sphere_fk(const ConfigurationBlock<rake> &q, Spheres<rake> &out) noexcept
    {
        out.x[0] = q[0];
        out.y[0] = q[1];
        out.z[0] = q[2];
        out.r[0] = radius;

    }

    template <std::size_t rake>
    inline bool interleaved_sphere_fk(
        const vamp::collision::Environment<FloatVector<rake>> &environment,
        const ConfigurationBlock<rake> &q) noexcept
    {
        return not sphere_environment_in_collision(environment, q[0], q[1],radius, radius);
    }

    // by Yaniv - forward propagation for a given initial configuration, control and duration
    inline auto
    dynamics(const Configuration &state, const Control &control_input, Configuration &dstate) noexcept
        -> Configuration
    {
        const float L = 0.1;

        const auto theta = state.data[0][3];
        const auto v = control_input.data[0][0];
        const auto steer = control_input.data[0][1];

        dstate.data[0][0] = cos(theta) * v;
        dstate.data[0][1] = sin(theta) * v;

        // dstate.data[0][2] = (v / L) * tan(steer); Adir - theta is 3 or 2?? suspected bug

        // ADIR - bug corrected, modifying theta instead of z

        dstate.data[0][2] = 0.0f;                         // (flat-ground assumption)
        dstate.data[0][3] = (v / L) * std::tan(steer);    // θ̇

        return dstate;
    }
    inline void
    fp_euler(Configuration &q, const Control &a, const float duration) noexcept
    {
        
        const float dt = duration < 0.01 ? duration : 0.01;  // min(duration, 0.01);
        Configuration dq;  

        for (float t = 0; t < duration; t+=dt)
        {
            dq = dynamics(q, a, dq);
            q = q + dq * dt;
            //return; // ****** used to test FP runtimes ******
        }
        q.data[0][3] = fmod(q.data[0][3], 2 * M_PI);  // [0,2*pi)
        if (q.data[0][3] < 0)
        {
            q.data[0][3] += 2 * M_PI;
        }
			
    }

    inline void fp_rk4(Configuration &q, const Control &a, const float duration) noexcept
    {
        const float dt = duration < 0.01 ? duration : 0.01;  // min(duration, 0.01);
        Configuration dq;

        // RK4 integration
        for (float t = 0; t < duration; t += dt)
        {
            auto k1 = dynamics(q, a, dq);
            auto k2 = dynamics(q + 0.5 * dt * k1, a, dq);
            auto k3 = dynamics(q + 0.5 * dt * k2, a, dq);
            auto k4 = dynamics(q + dt * k3, a, dq);

            q = q + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);
        }
        q.data[0][3] = fmod(q.data[0][3], 2 * M_PI);  // [0,2*pi)
        if (q.data[0][3] < 0)
        {
            q.data[0][3] += 2 * M_PI;
        }

    }
    inline auto eefk(const std::array<float, 4> &q) noexcept -> std::array<float, 7>
    {
    }
}  // namespace vamp::robots::ackermann

// NOLINTEND(*-magic-numbers)
