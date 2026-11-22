#ifndef Q_LEARNING_AGENT_H
#define Q_LEARNING_AGENT_H

#include "bitrl/base/bitrl_types.h"
#include "bitrl/rl/policies/epsilon_greedy_policy.h"
#include "bitsim/utils/concurrency/parallel.h"

#include <stdexcept>

namespace bitrl
{
namespace rl::multi_agent
{

///
/// \brief The settings of a Q-learning agent among others, and of IndependentQLearning
///
struct IndependentQLearningConfig
{
    real_t gamma{0.99};     ///< the discount factor
    real_t eta{0.1};        ///< the learning rate
    real_t eps_max{1.0};    ///< the exploration at the start
    real_t eps_min{0.05};   ///< the exploration it decays to
    real_t eps_decay{0.01}; ///< eps = eps_min + (eps_max - eps_min) exp(-eps_decay * episode)
    uint_t seed{42};        ///< every agent explores with its own generator, seeded from this
};

///
/// \brief One agent of a multi-agent environment that learns its own Q-table with Q-learning,
/// as if the others were part of the environment.
///
/// It reads its own observation, reward and done flag from the multi-agent time steps, and
/// explores with an epsilon-greedy policy whose generator is seeded with
/// bitsim::utils::concurrency::task_seed(config.seed, agent). IndependentQLearning is a team of
/// them; on its own, the agent can learn in a process of its own, see
/// bitrl::envs::multi_agent::DistributedAgentEnv, and learns there exactly what it learns in the
/// team.
///
class QLearningAgent
{
  public:
    typedef uint_t action_type;

    QLearningAgent(uint_t agent, uint_t n_states, uint_t n_actions,
                   const IndependentQLearningConfig &config = {});

    ///
    /// \brief The index of the agent in the time steps
    ///
    uint_t agent() const noexcept { return agent_; }

    ///
    /// \brief An exploring action; 0 if the agent is done
    ///
    template <typename TimeStepType> uint_t act(const TimeStepType &step);

    ///
    /// \brief The greedy action; 0 if the agent is done
    ///
    template <typename TimeStepType> uint_t act_greedy(const TimeStepType &step) const;

    ///
    /// \brief Learn from the transition step --action--> next; nothing once the agent is done
    ///
    template <typename TimeStepType>
    void update(const TimeStepType &step, uint_t action, const TimeStepType &next);

    void on_episode_begin(uint_t /*episode*/) {}

    ///
    /// \brief Decay the exploration
    ///
    void on_episode_end(uint_t episode) { policy_.on_episode(episode + 1); }

    real_t epsilon() const noexcept { return policy_.eps_value(); }

    const DynMat<real_t> &q_table() const noexcept { return q_table_; }

    ///
    /// \brief An actor that acts greedily with this table
    ///
    struct Greedy
    {
        const QLearningAgent &agent;
        template <typename TimeStepType> uint_t act(const TimeStepType &step) const
        {
            return agent.act_greedy(step);
        }
    };

    Greedy greedy() const { return Greedy{*this}; }

  private:
    uint_t agent_;
    IndependentQLearningConfig config_;
    DynMat<real_t> q_table_;
    policies::EpsilonGreedyPolicy policy_;
};

inline QLearningAgent::QLearningAgent(uint_t agent, uint_t n_states, uint_t n_actions,
                                      const IndependentQLearningConfig &config)
    : agent_(agent), config_(config),
      q_table_(DynMat<real_t>::Zero(static_cast<Eigen::Index>(n_states),
                                    static_cast<Eigen::Index>(n_actions))),
      // the agents must not explore in step, nor share a generator
      policy_(config.eps_max, bitsim::utils::concurrency::task_seed(config.seed, agent),
              policies::EpsilonDecayOption::EXPONENTIAL, config.eps_min, config.eps_max,
              config.eps_decay)
{
    if (n_states == 0 || n_actions == 0)
    {
        throw std::invalid_argument("QLearningAgent: needs states and actions");
    }
}

template <typename TimeStepType> uint_t QLearningAgent::act(const TimeStepType &step)
{
    return step.done(agent_) ? 0 : policy_(q_table_.row(step.observation(agent_)));
}

template <typename TimeStepType> uint_t QLearningAgent::act_greedy(const TimeStepType &step) const
{
    if (step.done(agent_))
    {
        return 0;
    }
    Eigen::Index best = 0;
    q_table_.row(step.observation(agent_)).maxCoeff(&best);
    return static_cast<uint_t>(best);
}

template <typename TimeStepType>
void QLearningAgent::update(const TimeStepType &step, uint_t action, const TimeStepType &next)
{
    if (step.done(agent_))
    {
        return;
    }
    const uint_t o = step.observation(agent_);
    const uint_t o_next = next.observation(agent_);
    const real_t bootstrap =
        next.done(agent_) ? 0.0 : config_.gamma * q_table_.row(o_next).maxCoeff();
    const real_t target = next.reward(agent_) + bootstrap;
    q_table_(o, action) += config_.eta * (target - q_table_(o, action));
}

} // namespace rl::multi_agent
} // namespace bitrl

#endif // Q_LEARNING_AGENT_H
