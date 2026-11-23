#include "bitrl/envs/classic_control/cart_pole_swing_up.h"

#include <cmath>
#include <random>
#include <stdexcept>

namespace bitrl
{
namespace envs::classic_control
{

namespace
{
template <typename T>
T option(const std::unordered_map<std::string, std::any> &options, const std::string &name,
         const T &value)
{
    const auto itr = options.find(name);
    return itr == options.end() ? value : std::any_cast<T>(itr->second);
}
} // namespace

CartPoleSwingUp::CartPoleSwingUp() : base_type("0", "CartPoleSwingUp") {}

void CartPoleSwingUp::make(const std::string &version,
                           const std::unordered_map<std::string, std::any> &make_options,
                           const std::unordered_map<std::string, std::any> &reset_options)
{
    base_type::make(version, make_options, reset_options);
    max_steps_ = option<uint_t>(make_options, "max_steps", 500);
    dt_ = option<real_t>(make_options, "dt", 0.02);
    off_track_penalty_ = option<real_t>(make_options, "off_track_penalty", 100.0);
    bitsim::dynamics::CartPoleParameters parameters;
    parameters.max_force = option<real_t>(make_options, "max_force", 10.0);
    if (max_steps_ == 0 || dt_ <= 0.0 || parameters.max_force <= 0.0 || off_track_penalty_ < 0.0)
    {
        throw std::invalid_argument("CartPoleSwingUp: max_steps, dt and max_force must be "
                                    "positive, off_track_penalty not negative");
    }
    dynamics_ = bitsim::dynamics::CartPoleDynamics(parameters);
    this->set_version_(version);
    this->make_created_();
}

void CartPoleSwingUp::close() { base_type::close(); }

real_t CartPoleSwingUp::angle() const
{
    const real_t theta = dynamics_.state()[2];
    return std::atan2(std::sin(theta), std::cos(theta));
}

CartPoleSwingUp::state_type CartPoleSwingUp::observe_() const
{
    const auto &x = dynamics_.state();
    return {x[0], x[1], std::sin(x[2]), std::cos(x[2]), x[3]};
}

CartPoleSwingUp::time_step_type CartPoleSwingUp::reset() { return reset(this->next_reset_seed_()); }

CartPoleSwingUp::time_step_type CartPoleSwingUp::reset(uint_t seed)
{
    if (!this->is_created())
    {
        throw std::logic_error("CartPoleSwingUp: call make() before reset()");
    }
    std::mt19937 rng(static_cast<std::mt19937::result_type>(seed));
    std::uniform_real_distribution<real_t> start(-0.05, 0.05);
    bitsim::dynamics::CartPoleState x;
    for (int i = 0; i < 4; ++i)
    {
        x[i] = start(rng);
    }
    x[2] += 3.14159265358979323846; // hanging down
    dynamics_ = bitsim::dynamics::CartPoleDynamics(dynamics_.parameters(), x);
    n_steps_ = 0;
    this->get_current_time_step_() = time_step_type(TimeStepTp::FIRST, 0.0, observe_());
    return this->get_current_time_step_();
}

CartPoleSwingUp::time_step_type CartPoleSwingUp::step(const action_type &action)
{
    if (!this->is_created())
    {
        throw std::logic_error("CartPoleSwingUp: call make() before step()");
    }
    if (action.size() != 1)
    {
        throw std::invalid_argument("CartPoleSwingUp: the action is one force");
    }
    dynamics_.step(action[0], dt_); // the dynamics limit it to max_force
    ++n_steps_;

    const auto &x = dynamics_.state();
    const bool off = dynamics_.off_track();
    const bool truncated = !off && n_steps_ >= max_steps_;
    const real_t reward =
        off ? -off_track_penalty_ : 0.5 * (1.0 + std::cos(x[2])) - 0.02 * x[0] * x[0];
    const bool upright = std::abs(angle()) < UPRIGHT;
    this->get_current_time_step_() = time_step_type(
        off || truncated ? TimeStepTp::LAST : TimeStepTp::MID, reward, observe_(), 1.0,
        {{"truncated", std::any(truncated)}, {"upright", std::any(upright)}});
    return this->get_current_time_step_();
}

} // namespace envs::classic_control
} // namespace bitrl
