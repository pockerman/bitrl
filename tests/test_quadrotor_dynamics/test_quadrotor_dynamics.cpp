#include "bitrl/bitrl_types.h"
#include "bitrl/dynamics/quadrotor_dynamics.h"
#include "bitrl/dynamics/system_state.h"

#include <gtest/gtest.h>

#include <array>
#include <string>
#include <utility>

namespace
{

using bitrl::real_t;
using bitrl::RealColVec3d;
using bitrl::RealVec;
using bitrl::dynamics::QuadrotorDynamics;
using bitrl::dynamics::QuadrotorDynamicsConfig;
using bitrl::dynamics::SysState;

const real_t DT = 0.001;

QuadrotorDynamicsConfig make_config()
{
    QuadrotorDynamicsConfig config;
    config.dt = DT;
    config.mass = 0.468;
    config.l = 0.225;
    config.k_1 = 2.980e-6;
    config.k_2 = 1.140e-7;
    config.Jx = 4.856e-3;
    config.Jy = 4.856e-3;
    config.Jz = 8.801e-3;
    config.use_gravity = false;
    return config;
}

SysState<12> make_state(real_t v, real_t phi, real_t theta, real_t psi)
{
    std::array<std::pair<std::string, real_t>, 12> values;
    values[0] = {"x", 0.0};
    values[1] = {"y", 0.0};
    values[2] = {"z", 0.0};
    values[3] = {"u", 0.0};
    values[4] = {"v", v};
    values[5] = {"w", 0.0};
    values[6] = {"p", 0.0};
    values[7] = {"q", 0.0};
    values[8] = {"r", 0.0};
    values[9] = {"phi", phi};
    values[10] = {"theta", theta};
    values[11] = {"psi", psi};
    return SysState<12>(std::move(values));
}

} // namespace

// The position advances by dt * R * v. R is orthogonal so the step has length dt * |v|
// whatever the attitude is. No motors, gravity or rotation, so the attitude stays fixed
TEST(TestQuadrotorDynamics, PositionStepPreservesTheLengthOfTheVelocity)
{
    const real_t speed = 2.0;
    auto state = make_state(speed, 0.3, 0.2, -0.4);
    QuadrotorDynamics dynamics(make_config(), state);

    dynamics.integrate(RealVec::Zero(4));

    EXPECT_NEAR(dynamics.get_position().norm(), DT * speed, 1.0e-12);
}
