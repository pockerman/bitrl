#ifndef DISTRIBUTED_AGENT_ENV_H
#define DISTRIBUTED_AGENT_ENV_H

#include "bitrl/base/bitrl_types.h"
#include "bitsim/distributed/synchroniser.h"
#include "bitsim/distributed/transport.h"

#include <array>
#include <bit>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace bitrl
{
namespace envs::multi_agent
{

///
/// \brief How an action or an observation travels as numbers, for DistributedAgentEnv.
///
/// Numbers and enums are one number; std::array and std::vector of numbers are their
/// elements. Specialise it for other types.
///
template <typename T, typename Enable = void> struct AgentValues;

template <typename T>
struct AgentValues<T, std::enable_if_t<std::is_arithmetic_v<T> || std::is_enum_v<T>>>
{
    static void append(const T &value, std::vector<real_t> &out)
    {
        out.push_back(static_cast<real_t>(value));
    }
    static T read(const std::vector<real_t> &values, std::size_t first, std::size_t /*count*/)
    {
        if constexpr (std::is_enum_v<T>)
        {
            return static_cast<T>(static_cast<std::underlying_type_t<T>>(values.at(first)));
        }
        else
        {
            return static_cast<T>(values.at(first));
        }
    }
};

template <typename T, std::size_t N> struct AgentValues<std::array<T, N>>
{
    static void append(const std::array<T, N> &value, std::vector<real_t> &out)
    {
        for (const auto &v : value)
        {
            out.push_back(static_cast<real_t>(v));
        }
    }
    static std::array<T, N> read(const std::vector<real_t> &values, std::size_t first,
                                 std::size_t count)
    {
        if (count != N)
        {
            throw std::invalid_argument("AgentValues: expected " + std::to_string(N) +
                                        " values, got " + std::to_string(count));
        }
        std::array<T, N> a{};
        for (std::size_t i = 0; i < N; ++i)
        {
            a[i] = static_cast<T>(values.at(first + i));
        }
        return a;
    }
};

template <typename T> struct AgentValues<std::vector<T>>
{
    static void append(const std::vector<T> &value, std::vector<real_t> &out)
    {
        for (const auto &v : value)
        {
            out.push_back(static_cast<real_t>(v));
        }
    }
    static std::vector<T> read(const std::vector<real_t> &values, std::size_t first,
                               std::size_t count)
    {
        std::vector<T> v(count);
        for (std::size_t i = 0; i < count; ++i)
        {
            v[i] = static_cast<T>(values.at(first + i));
        }
        return v;
    }
};

///
/// \brief A checksum of an observation that a real_t holds exactly, to tell that two copies of
/// an environment agree.
///
template <typename T> real_t observation_checksum(const T &observation)
{
    std::vector<real_t> values;
    AgentValues<T>::append(observation, values);
    std::uint64_t h = 1469598103934665603ULL; // FNV-1a
    for (const real_t v : values)
    {
        const auto bits = std::bit_cast<std::uint64_t>(v + 0.0); // -0 and 0 alike
        for (int b = 0; b < 64; b += 8)
        {
            h ^= (bits >> b) & 0xFFu;
            h *= 1099511628211ULL;
        }
    }
    return static_cast<real_t>(h & ((std::uint64_t(1) << 53) - 1));
}

///
/// \brief The settings of DistributedAgentEnv
///
struct DistributedAgentConfig
{
    real_t timeout{10.0};       ///< how long a step waits for the others [s]
    real_t resend_period{0.02}; ///< how often a waiting step sends its last steps again [s]
    bool check_replicas{true};  ///< throw if the copies of the environment differ
};

///
/// \brief One agent of a multi-agent environment, in a process of its own.
///
/// Every agent of the environment is a participant with its own DistributedAgentEnv: a thread,
/// a process or a machine, meeting the others through a bitsim::distributed::Transport. Every
/// participant holds a copy of the whole environment, made with the same options, and only the
/// actions travel: step() sends this agent's action, waits for the actions of the others at the
/// same step, with bitsim::distributed::Synchroniser in lockstep, and steps its copy with the
/// joint action. The copies start alike and get the same joint actions, so they stay alike, and
/// the episodes are the same as in one process. That suits environments that are cheap next to
/// the agents, such as a learner or a controller per robot, or a real robot's on-board
/// computer, and that are deterministic; bitsim's distributed simulation, where every
/// participant simulates only its own robots, suits worlds that are not.
///
/// With check_replicas, every action comes with a checksum of the observation its agent acted
/// on, and step() throws if it is not the observation of that agent in this copy: an
/// environment that is not deterministic, made with other options, or reset with another seed.
///
/// The time steps are those of the copy, so they hold the observation and the reward of every
/// agent; a decentralised agent reads only its own, observation(agent()).
///
/// \tparam EnvType a bitrl::envs::multi_agent::MultiAgentEnvBase
///
template <typename EnvType> class DistributedAgentEnv
{
  public:
    typedef EnvType env_type;
    typedef typename EnvType::time_step_type time_step_type;
    typedef typename EnvType::action_type action_type;
    typedef typename EnvType::observation_type observation_type;
    typedef typename EnvType::joint_action_type joint_action_type;

    ///
    /// \param env the copy of the environment, made; it must outlive this
    /// \param agent the agent this participant is
    /// \param transport where the participants meet; it must outlive this
    ///
    DistributedAgentEnv(EnvType &env, uint_t agent, bitsim::distributed::Transport &transport,
                        const DistributedAgentConfig &config = {});

    ///
    /// \brief Start a new episode with the next seed of the copy
    ///
    time_step_type reset() { return remember_(env_.reset()); }

    ///
    /// \brief Start a new episode with seed; every participant must use the same
    ///
    time_step_type reset(uint_t seed) { return remember_(env_.reset(seed)); }

    ///
    /// \brief Send this agent's action, wait for the others', and step the copy
    /// \throws std::runtime_error if a peer did not send the step within timeout, or with
    /// check_replicas if the copies differ
    ///
    time_step_type step(const action_type &action);

    ///
    /// \brief Tell the others that this participant stops
    ///
    void leave() { sync_.leave(); }

    uint_t agent() const noexcept { return agent_; }

    uint_t n_agents() const noexcept { return env_.n_agents(); }

    const EnvType &env() const noexcept { return env_; }

    const bitsim::distributed::Synchroniser &synchroniser() const noexcept { return sync_; }

  private:
    static bitsim::distributed::SynchroniserConfig sync_config_(const EnvType &env, uint_t agent,
                                                                const DistributedAgentConfig &c);

    time_step_type remember_(time_step_type step);

    EnvType &env_;
    uint_t agent_;
    DistributedAgentConfig config_;
    bitsim::distributed::Synchroniser sync_;
    std::vector<real_t> checksums_; ///< of every agent's current observation
    uint_t n_exchanges_{0};
    bool started_{false};
};

template <typename EnvType>
bitsim::distributed::SynchroniserConfig
DistributedAgentEnv<EnvType>::sync_config_(const EnvType &env, uint_t agent,
                                           const DistributedAgentConfig &c)
{
    if (!env.is_created())
    {
        throw std::logic_error("DistributedAgentEnv: make() the environment first");
    }
    if (agent >= env.n_agents())
    {
        throw std::invalid_argument("DistributedAgentEnv: no agent " + std::to_string(agent));
    }
    bitsim::distributed::SynchroniserConfig s;
    s.mode = bitsim::distributed::SyncMode::LOCKSTEP;
    s.timeout = c.timeout;
    s.resend_period = c.resend_period;
    for (uint_t a = 0; a < env.n_agents(); ++a)
    {
        if (a != agent)
        {
            s.peers.push_back(env.agent_name(a));
        }
    }
    return s;
}

template <typename EnvType>
DistributedAgentEnv<EnvType>::DistributedAgentEnv(EnvType &env, uint_t agent,
                                                  bitsim::distributed::Transport &transport,
                                                  const DistributedAgentConfig &config)
    : env_(env), agent_(agent), config_(config),
      sync_(transport, env.agent_name(agent), sync_config_(env, agent, config))
{
}

template <typename EnvType>
typename DistributedAgentEnv<EnvType>::time_step_type
DistributedAgentEnv<EnvType>::remember_(time_step_type step)
{
    started_ = true;
    checksums_.resize(step.n_agents());
    for (uint_t a = 0; a < step.n_agents(); ++a)
    {
        checksums_[a] = config_.check_replicas ? observation_checksum(step.observation(a)) : 0.0;
    }
    return step;
}

template <typename EnvType>
typename DistributedAgentEnv<EnvType>::time_step_type
DistributedAgentEnv<EnvType>::step(const action_type &action)
{
    if (!started_)
    {
        throw std::logic_error("DistributedAgentEnv: call reset() before step()");
    }
    // the action, then the checksum of what it was chosen from
    std::vector<real_t> values;
    AgentValues<action_type>::append(action, values);
    values.push_back(checksums_[agent_]);

    const auto &peers = sync_.exchange(n_exchanges_++, {}, values);

    joint_action_type actions(n_agents());
    actions[agent_] = action;
    for (const auto &peer : peers)
    {
        uint_t a = 0;
        while (a < n_agents() && env_.agent_name(a) != peer.participant)
        {
            ++a;
        }
        // a peer that left at this step sent it with its leave; one that left before did not
        if (peer.step != n_exchanges_ - 1 || peer.values.empty())
        {
            throw std::runtime_error("DistributedAgentEnv " + env_.agent_name(agent_) + ": " +
                                     peer.participant + " has left");
        }
        if (config_.check_replicas && peer.values.back() != checksums_[a])
        {
            throw std::runtime_error("DistributedAgentEnv " + env_.agent_name(agent_) +
                                     ": the copies of the environment differ; " + peer.participant +
                                     " observed something else at step " +
                                     std::to_string(n_exchanges_ - 1));
        }
        actions[a] = AgentValues<action_type>::read(peer.values, 0, peer.values.size() - 1);
    }
    return remember_(env_.step(actions));
}

} // namespace envs::multi_agent
} // namespace bitrl

#endif // DISTRIBUTED_AGENT_ENV_H
