#ifndef MULTI_AGENT_TIME_STEP_H
#define MULTI_AGENT_TIME_STEP_H

#include "bitrl/base/bitrl_types.h"
#include "bitrl/envs/time_step_type.h"

#include <any>
#include <numeric>
#include <ostream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bitrl
{
namespace envs::multi_agent
{

///
/// \brief What a multi-agent environment returns after a reset or a joint step: an observation,
/// a reward and a done flag for every agent.
///
/// An agent is done when it finished its own task, a robot that reached its goal for example;
/// the environment ignores its actions from then on. The episode is over, last(), when every
/// agent is done, or when the environment stopped it at its step limit, truncated(). Agents are
/// numbered 0, 1, ..., n_agents() - 1 in every vector.
///
template <typename ObservationType> class MultiAgentTimeStep
{
  public:
    typedef ObservationType observation_type;

    MultiAgentTimeStep() = default;

    ///
    /// \brief A time step of n agents, with every vector of size n
    ///
    MultiAgentTimeStep(TimeStepTp type, std::vector<observation_type> observations,
                       std::vector<real_t> rewards, std::vector<bool> dones, bool truncated = false,
                       std::unordered_map<std::string, std::any> extra = {});

    ///
    /// \brief The type of the step: FIRST after a reset, LAST once the episode is over
    ///
    TimeStepTp type() const noexcept { return type_; }

    bool first() const noexcept { return type_ == TimeStepTp::FIRST; }

    bool mid() const noexcept { return type_ == TimeStepTp::MID; }

    ///
    /// \brief Whether the episode is over
    ///
    bool last() const noexcept { return type_ == TimeStepTp::LAST; }

    ///
    /// \brief Whether the step limit ended the episode before every agent was done
    ///
    bool truncated() const noexcept { return truncated_; }

    uint_t n_agents() const noexcept { return static_cast<uint_t>(observations_.size()); }

    const observation_type &observation(uint_t agent) const { return observations_.at(agent); }

    const std::vector<observation_type> &observations() const noexcept { return observations_; }

    real_t reward(uint_t agent) const { return rewards_.at(agent); }

    const std::vector<real_t> &rewards() const noexcept { return rewards_; }

    ///
    /// \brief The sum of the rewards of the agents, the team's reward
    ///
    real_t total_reward() const noexcept
    {
        return std::accumulate(rewards_.begin(), rewards_.end(), real_t(0));
    }

    bool done(uint_t agent) const { return dones_.at(agent); }

    const std::vector<bool> &dones() const noexcept { return dones_; }

    ///
    /// \brief Anything else the environment reports, collisions for example
    ///
    const std::unordered_map<std::string, std::any> &extra() const noexcept { return extra_; }

    ///
    /// \brief Read an entry of extra()
    ///
    template <typename T> T get_extra(const std::string &name) const;

  private:
    TimeStepTp type_{TimeStepTp::INVALID_TYPE};
    std::vector<observation_type> observations_;
    std::vector<real_t> rewards_;
    std::vector<bool> dones_;
    bool truncated_{false};
    std::unordered_map<std::string, std::any> extra_;
};

template <typename ObservationType>
MultiAgentTimeStep<ObservationType>::MultiAgentTimeStep(
    TimeStepTp type, std::vector<observation_type> observations, std::vector<real_t> rewards,
    std::vector<bool> dones, bool truncated, std::unordered_map<std::string, std::any> extra)
    : type_(type), observations_(std::move(observations)), rewards_(std::move(rewards)),
      dones_(std::move(dones)), truncated_(truncated), extra_(std::move(extra))
{
    if (rewards_.size() != observations_.size() || dones_.size() != observations_.size())
    {
        throw std::invalid_argument("MultiAgentTimeStep: every agent needs an observation, a "
                                    "reward and a done flag");
    }
}

template <typename ObservationType>
template <typename T>
T MultiAgentTimeStep<ObservationType>::get_extra(const std::string &name) const
{
    const auto itr = extra_.find(name);
    if (itr == extra_.end())
    {
        throw std::logic_error("MultiAgentTimeStep: no extra entry " + name);
    }
    return std::any_cast<T>(itr->second);
}

template <typename ObservationType>
std::ostream &operator<<(std::ostream &out, const MultiAgentTimeStep<ObservationType> &step)
{
    out << "MultiAgentTimeStep(" << TimeStepEnumUtils::to_string(step.type())
        << ", agents=" << step.n_agents() << ", rewards=[";
    for (uint_t a = 0; a < step.n_agents(); ++a)
    {
        out << (a ? ", " : "") << step.reward(a);
    }
    out << "], done=[";
    for (uint_t a = 0; a < step.n_agents(); ++a)
    {
        out << (a ? ", " : "") << (step.done(a) ? "yes" : "no");
    }
    out << "]" << (step.truncated() ? ", truncated" : "") << ")";
    return out;
}

} // namespace envs::multi_agent
} // namespace bitrl

#endif // MULTI_AGENT_TIME_STEP_H
