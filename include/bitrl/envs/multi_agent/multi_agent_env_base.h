#ifndef MULTI_AGENT_ENV_BASE_H
#define MULTI_AGENT_ENV_BASE_H

#include "bitrl/base/bitrl_types.h"
#include "bitrl/envs/multi_agent/multi_agent_time_step.h"
#include "bitsim/utils/std_map_utils.h"

#include <any>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace bitrl
{
namespace envs::multi_agent
{

///
/// \brief The base of environments where several agents act at the same time.
///
/// Every step takes a joint action, one action per agent, and the environment applies them
/// together; this is the "parallel" form of multi-agent environments that PettingZoo also uses.
/// A turn-based game fits it too, with the agents whose turn it is not ignored.
///
/// The interface follows bitrl::envs::EnvBase: make() with options, reset() and step(). Every
/// reset() uses the next seed, seed + n, where seed is the reset option "seed" (42 if not given)
/// and n the number of resets since make(), so that episodes start differently but repeatably.
///
/// \tparam ObservationType what one agent observes
/// \tparam ActionType what one agent does
///
template <typename ObservationType, typename ActionType> class MultiAgentEnvBase
{
  public:
    typedef ObservationType observation_type;
    typedef ActionType action_type;
    typedef MultiAgentTimeStep<ObservationType> time_step_type;

    ///
    /// \brief One action per agent, in the order of the agents
    ///
    typedef std::vector<action_type> joint_action_type;

    static constexpr uint_t DEFAULT_ENV_SEED = 42;

    virtual ~MultiAgentEnvBase() = default;

    ///
    /// \brief Build the environment
    /// \param version the version of the environment
    /// \param make_options the options of the environment; see the derived class
    /// \param reset_options the options of reset(), "seed" for one
    ///
    virtual void make(const std::string &version,
                      const std::unordered_map<std::string, std::any> &make_options,
                      const std::unordered_map<std::string, std::any> &reset_options);

    ///
    /// \brief Start a new episode with the next seed
    ///
    time_step_type reset() { return reset(next_reset_seed_()); }

    ///
    /// \brief Start a new episode with the given seed
    ///
    virtual time_step_type reset(uint_t seed) = 0;

    ///
    /// \brief Apply one action per agent
    ///
    virtual time_step_type step(const joint_action_type &actions) = 0;

    ///
    /// \brief Release the environment; make() builds it again
    ///
    virtual void close() { is_created_ = false; }

    uint_t n_agents() const noexcept { return static_cast<uint_t>(agent_names_.size()); }

    ///
    /// \brief The name of an agent, for printing
    ///
    const std::string &agent_name(uint_t agent) const { return agent_names_.at(agent); }

    bool is_created() const noexcept { return is_created_; }

    std::string env_name() const noexcept { return name_; }

    std::string version() const noexcept { return version_; }

    ///
    /// \brief The steps of the current episode so far
    ///
    uint_t n_steps() const noexcept { return n_steps_; }

    ///
    /// \brief The steps after which an episode is truncated
    ///
    uint_t max_steps() const noexcept { return max_steps_; }

    uint_t last_reset_seed() const noexcept { return last_reset_seed_; }

    const std::unordered_map<std::string, std::any> &make_options() const noexcept
    {
        return make_options_;
    }

  protected:
    explicit MultiAgentEnvBase(const std::string &name) : name_(name) {}

    ///
    /// \brief Name the agents; their number is the number of names
    ///
    void set_agents_(std::vector<std::string> names) { agent_names_ = std::move(names); }

    void set_max_steps_(uint_t max_steps) noexcept { max_steps_ = max_steps; }

    void make_created_() noexcept { is_created_ = true; }

    ///
    /// \brief Start counting the steps of a new episode, for reset()
    ///
    void start_episode_() noexcept { n_steps_ = 0; }

    ///
    /// \brief Count a step, for step()
    /// \return whether the episode reached its step limit
    ///
    bool count_step_() noexcept { return ++n_steps_ >= max_steps_; }

    ///
    /// \brief Throw unless make() was called and actions has one action per agent
    ///
    void check_step_(const joint_action_type &actions) const;

    ///
    /// \brief Read a make option, or give default_value when it is missing
    ///
    template <typename T> T option_(const std::string &name, const T &default_value) const;

  private:
    uint_t next_reset_seed_();

    std::string name_;
    std::string version_;
    std::vector<std::string> agent_names_;
    std::unordered_map<std::string, std::any> make_options_;
    std::unordered_map<std::string, std::any> reset_options_;
    bool is_created_{false};
    uint_t n_steps_{0};
    uint_t max_steps_{1000};
    uint_t n_resets_{0};
    uint_t last_reset_seed_{DEFAULT_ENV_SEED};
};

template <typename ObservationType, typename ActionType>
void MultiAgentEnvBase<ObservationType, ActionType>::make(
    const std::string &version, const std::unordered_map<std::string, std::any> &make_options,
    const std::unordered_map<std::string, std::any> &reset_options)
{
    version_ = version;
    make_options_ = make_options;
    reset_options_ = reset_options;
    n_resets_ = 0;
    max_steps_ = option_<uint_t>("max_steps", max_steps_);
}

template <typename ObservationType, typename ActionType>
uint_t MultiAgentEnvBase<ObservationType, ActionType>::next_reset_seed_()
{
    const uint_t seed = reset_options_.contains("seed")
                            ? bitsim::utils::resolve<uint_t>("seed", reset_options_)
                            : DEFAULT_ENV_SEED;
    last_reset_seed_ = seed + n_resets_++;
    return last_reset_seed_;
}

template <typename ObservationType, typename ActionType>
void MultiAgentEnvBase<ObservationType, ActionType>::check_step_(
    const joint_action_type &actions) const
{
    if (!is_created_)
    {
        throw std::logic_error(name_ + ": call make() before step()");
    }
    if (actions.size() != n_agents())
    {
        throw std::invalid_argument(name_ + ": step() needs one action per agent, " +
                                    std::to_string(n_agents()) + " actions, but got " +
                                    std::to_string(actions.size()));
    }
}

template <typename ObservationType, typename ActionType>
template <typename T>
T MultiAgentEnvBase<ObservationType, ActionType>::option_(const std::string &name,
                                                          const T &default_value) const
{
    const auto itr = make_options_.find(name);
    return itr == make_options_.end() ? default_value : std::any_cast<T>(itr->second);
}

} // namespace envs::multi_agent
} // namespace bitrl

#endif // MULTI_AGENT_ENV_BASE_H
