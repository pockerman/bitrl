#include "bitrl/envs/classic_control/cart_pole.h"

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

CartPole::CartPole() : base_type("0", "CartPole") {}

void CartPole::make(const std::string &version,
                    const std::unordered_map<std::string, std::any> &make_options,
                    const std::unordered_map<std::string, std::any> &reset_options)
{
    base_type::make(version, make_options, reset_options);
    max_steps_ = option<uint_t>(make_options, "max_steps", 500);
    dt_ = option<real_t>(make_options, "dt", 0.02);
    bitsim::dynamics::CartPoleParameters parameters;
    parameters.max_force = option<real_t>(make_options, "max_force", 10.0);
    if (max_steps_ == 0 || dt_ <= 0.0 || parameters.max_force <= 0.0)
    {
        throw std::invalid_argument("CartPole: max_steps, dt and max_force must be positive");
    }
    dynamics_ = bitsim::dynamics::CartPoleDynamics(parameters);
    this->set_version_(version);
    this->make_created_();
}

void CartPole::close() { base_type::close(); }

CartPole::state_type CartPole::observe_() const
{
    const auto &x = dynamics_.state();
    return {x[0], x[1], x[2], x[3]};
}

CartPole::time_step_type CartPole::reset() { return reset(this->next_reset_seed_()); }

CartPole::time_step_type CartPole::reset(uint_t seed)
{
    if (!this->is_created())
    {
        throw std::logic_error("CartPole: call make() before reset()");
    }
    std::mt19937 rng(static_cast<std::mt19937::result_type>(seed));
    std::uniform_real_distribution<real_t> start(-0.05, 0.05);
    bitsim::dynamics::CartPoleState x;
    for (int i = 0; i < 4; ++i)
    {
        x[i] = start(rng);
    }
    dynamics_ = bitsim::dynamics::CartPoleDynamics(dynamics_.parameters(), x);
    n_steps_ = 0;
    this->get_current_time_step_() = time_step_type(TimeStepTp::FIRST, 0.0, observe_());
    return this->get_current_time_step_();
}

CartPole::time_step_type CartPole::step(const action_type &action)
{
    if (!this->is_created())
    {
        throw std::logic_error("CartPole: call make() before step()");
    }
    if (action > 1)
    {
        throw std::invalid_argument("CartPole: the action must be 0 or 1");
    }
    const real_t force =
        action == 1 ? dynamics_.parameters().max_force : -dynamics_.parameters().max_force;
    dynamics_.step(force, dt_);
    ++n_steps_;

    const auto &x = dynamics_.state();
    const bool failed = dynamics_.off_track() || std::abs(x[2]) > THETA_LIMIT;
    const bool truncated = !failed && n_steps_ >= max_steps_;
    this->get_current_time_step_() =
        time_step_type(failed || truncated ? TimeStepTp::LAST : TimeStepTp::MID, 1.0, observe_(),
                       1.0, {{"truncated", std::any(truncated)}});
    return this->get_current_time_step_();
}

} // namespace envs::classic_control
} // namespace bitrl
