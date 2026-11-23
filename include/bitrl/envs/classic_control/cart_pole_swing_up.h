#ifndef BITRL_CLASSIC_CONTROL_CART_POLE_SWING_UP_H
#define BITRL_CLASSIC_CONTROL_CART_POLE_SWING_UP_H

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
/// \brief Swing the pole of a cart-pole up from hanging down, and keep it up, with a force that
/// can take any value, simulated with bitsim::dynamics::CartPoleDynamics.
///
/// The cart and the pole are those of CartPole, but the pole starts hanging down, and no push
/// of max_force lifts it at once: the cart has to pump energy into the swing, back and forth,
/// and then catch the pole at the top and balance it, all within the track. The action is the
/// force on the cart [N], a vector of one value, limited to [-max_force, max_force].
///
/// The observation is [x, x_dot, sin(theta), cos(theta), theta_dot], theta from upright. Every
/// step is worth (1 + cos(theta)) / 2, from 0 hanging down to 1 upright, minus 0.02 x^2 to keep
/// the cart near the centre. An episode starts with the pole within 0.05 rad of hanging down
/// and every other value uniform in [-0.05, 0.05], drawn with the seed of the reset. It ends
/// when the cart leaves the track, which costs off_track_penalty, or after max_steps steps; then
/// the time step's extra "truncated" is true. The extra "upright" says whether the pole is
/// within 12 degrees of upright.
///
/// Make options, all optional: "max_steps" (uint_t, 500), "dt" (real_t, 0.02), "max_force"
/// (real_t, 10.0), "off_track_penalty" (real_t, 100.0). Reset option: "seed" (uint_t), the
/// seed of the first reset; every reset after it takes the next seed.
///
class CartPoleSwingUp final
    : public EnvBase<TimeStep<std::vector<real_t>>,
                     ContinuousVectorStateContinuousVectorActionEnv<5, 1, real_t, real_t>>
{
  public:
    typedef EnvBase<TimeStep<std::vector<real_t>>,
                    ContinuousVectorStateContinuousVectorActionEnv<5, 1, real_t, real_t>>
        base_type;
    typedef typename base_type::time_step_type time_step_type;
    typedef typename base_type::state_type state_type;
    typedef typename base_type::action_type action_type;

    ///
    /// \brief Within this of upright [rad] the pole counts as up
    ///
    static constexpr real_t UPRIGHT = 12.0 * 3.14159265358979323846 / 180.0;

    CartPoleSwingUp();

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
    /// \brief Push the cart with action[0] newtons, positive towards +x
    ///
    time_step_type step(const action_type &action) override;

    uint_t n_steps() const noexcept { return n_steps_; }

    uint_t max_steps() const noexcept { return max_steps_; }

    real_t dt() const noexcept { return dt_; }

    ///
    /// \brief The angle of the pole from upright, in (-pi, pi]
    ///
    real_t angle() const;

    const bitsim::dynamics::CartPoleDynamics &dynamics() const noexcept { return dynamics_; }

  private:
    state_type observe_() const;

    bitsim::dynamics::CartPoleDynamics dynamics_;
    real_t dt_{0.02};
    real_t off_track_penalty_{100.0};
    uint_t max_steps_{500};
    uint_t n_steps_{0};
};

} // namespace envs::classic_control
} // namespace bitrl

#endif // BITRL_CLASSIC_CONTROL_CART_POLE_SWING_UP_H
