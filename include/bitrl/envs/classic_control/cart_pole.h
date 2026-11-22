#ifndef BITRL_CLASSIC_CONTROL_CART_POLE_H
#define BITRL_CLASSIC_CONTROL_CART_POLE_H

#include "bitrl/base/bitrl_types.h"
#include "bitrl/envs/env_base.h"
#include "bitrl/envs/env_types.h"
#include "bitrl/envs/time_step.h"
#include "bitsim/dynamics/cart_pole_dynamics.h"

#include <any>
#include <string>
#include <unordered_map>
#include <vector>

namespace bitrl
{
namespace envs::classic_control
{

///
/// \brief The cart-pole of Barto, Sutton and Anderson, simulated in this process with
/// bitsim::dynamics::CartPoleDynamics.
///
/// It is the task of Gymnasium's CartPole, without a server: push the cart left (action 0) or
/// right (action 1) with max_force, every dt seconds, and keep the pole within 12 degrees of
/// upright and the cart within 2.4 m of the centre. The observation is [x, x_dot, theta,
/// theta_dot]; every step is worth 1, the step that fails included. An episode starts with every
/// value uniform in [-0.05, 0.05], drawn with the seed of the reset, and ends when the pole falls,
/// the cart leaves the track, or after max_steps steps; then the time step's extra
/// "truncated" is true.
///
/// Unlike Gymnasium, which takes one Euler step, the dynamics integrate with fourth order
/// Runge-Kutta, so the numbers are close to Gymnasium's but not the same.
///
/// Make options, all optional: "max_steps" (uint_t, 500), "dt" (real_t, 0.02), "max_force"
/// (real_t, 10.0). Reset option: "seed" (uint_t), the seed of the first reset; every reset after
/// it takes the next seed.
///
class CartPole final : public EnvBase<TimeStep<std::vector<real_t>>,
                                      ContinuousVectorStateDiscreteActionEnv<4, 2, 0, real_t>>
{
  public:
    typedef EnvBase<TimeStep<std::vector<real_t>>,
                    ContinuousVectorStateDiscreteActionEnv<4, 2, 0, real_t>>
        base_type;
    typedef typename base_type::time_step_type time_step_type;
    typedef typename base_type::state_type state_type;
    typedef typename base_type::action_type action_type;

    static constexpr real_t THETA_LIMIT = 12.0 * 3.14159265358979323846 / 180.0;

    CartPole();

    void make(const std::string &version,
              const std::unordered_map<std::string, std::any> &make_options,
              const std::unordered_map<std::string, std::any> &reset_options) override;

    void close() override;

    ///
    /// \brief Start an episode with the next seed
    ///
    time_step_type reset() override;

    ///
    /// \brief Start an episode with the given seed
    ///
    time_step_type reset(uint_t seed);

    ///
    /// \brief Push the cart: 0 to the left, 1 to the right
    ///
    time_step_type step(const action_type &action) override;

    uint_t n_steps() const noexcept { return n_steps_; }

    uint_t max_steps() const noexcept { return max_steps_; }

    const bitsim::dynamics::CartPoleDynamics &dynamics() const noexcept { return dynamics_; }

  private:
    state_type observe_() const;

    bitsim::dynamics::CartPoleDynamics dynamics_;
    real_t dt_{0.02};
    uint_t max_steps_{500};
    uint_t n_steps_{0};
};

} // namespace envs::classic_control
} // namespace bitrl

#endif // BITRL_CLASSIC_CONTROL_CART_POLE_H
