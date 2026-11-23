#ifndef INDEPENDENT_Q_LEARNING_H
#define INDEPENDENT_Q_LEARNING_H

#include "bitrl/base/bitrl_types.h"
#include "bitrl/rl/policies/epsilon_greedy_policy.h"
#include "bitsim/utils/concurrency/parallel.h"
#include "bitrl/rl/multi_agent/q_learning_agent.h"
#include "bitsim/utils/io/csv_file_writer.h"

#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace bitrl
{
namespace rl::multi_agent
{

///
/// \brief The settings of IndependentQLearning
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
/// \brief Every agent learns its own Q-table with Q-learning, as if the others were part of the
/// environment.
///
/// Independent Q-learning is the simplest way to learn with several agents, and the baseline of
/// the others. Each agent explores with its own epsilon-greedy policy,
/// bitrl::rl::policies::EpsilonGreedyPolicy, and updates
/// \f$Q_i(o_i, a_i) \leftarrow Q_i(o_i, a_i) + \eta [r_i + \gamma \max_a Q_i(o_i', a) - Q_i(o_i,
/// a_i)]\f$ from its own observation, action and reward. The others change their behaviour while
/// they learn, so the environment of each agent is not stationary and the guarantees of Q-learning
/// are lost; how much that matters depends on what the agents observe of each other.
///
/// An agent that is done takes no actions and learns nothing more. Where the episode was cut by
/// the step limit, the agent still bootstraps from its next observation, since it was not done.
///
/// Every agent is a QLearningAgent, which can also learn on its own, in a process of its own.
///
/// It suits environments whose observations and actions are indices, such as
/// bitrl::envs::multi_agent::MultiAgentGridWorld, and meets the multi_agent_learner concept of
/// MultiAgentTrainer.
///
class IndependentQLearning
{
  public:
    typedef std::vector<uint_t> joint_action_type;

    IndependentQLearning(uint_t n_agents, uint_t n_states, uint_t n_actions,
                         const IndependentQLearningConfig &config = {});

    uint_t n_agents() const noexcept { return static_cast<uint_t>(q_tables_.size()); }
    uint_t n_agents() const noexcept { return static_cast<uint_t>(agents_.size()); }

    ///
    /// \brief An exploring action for every agent; agents that are done get action 0
    ///
    template <typename TimeStepType> joint_action_type act(const TimeStepType &step);

    ///
    /// \brief The greedy action of every agent, for evaluation
    ///
    template <typename TimeStepType> joint_action_type act_greedy(const TimeStepType &step) const;

    ///
    /// \brief Learn from the transition step --actions--> next
    ///
    template <typename TimeStepType>
    void update(const TimeStepType &step, const joint_action_type &actions,
                const TimeStepType &next);

    void on_episode_begin(uint_t /*episode*/) {}

    ///
    /// \brief Decay the exploration of every agent
    ///
    void on_episode_end(uint_t episode);

    ///
    /// \brief The current exploration, the same for all agents
    ///
    real_t epsilon() const noexcept { return policies_.front().eps_value(); }

    const DynMat<real_t> &q_table(uint_t agent) const { return q_tables_.at(agent); }
    real_t epsilon() const noexcept { return agents_.front().epsilon(); }

    const DynMat<real_t> &q_table(uint_t agent) const { return agents_.at(agent).q_table(); }

    ///
    /// \brief The learner of one agent
    ///
    const QLearningAgent &agent(uint_t agent) const { return agents_.at(agent); }

    ///
    /// \brief Write the Q-table of every agent to <prefix><agent>.csv, a row per state
    ///
    void save(const std::string &prefix) const;

    ///
    /// \brief A team that acts greedily with these tables, for run_episode()
    ///
    struct Greedy
    {
        const IndependentQLearning &learner;
        template <typename TimeStepType> joint_action_type act(const TimeStepType &step) const
        {
            return learner.act_greedy(step);
        }
    };

    Greedy greedy() const { return Greedy{*this}; }

  private:
    IndependentQLearningConfig config_;
    std::vector<DynMat<real_t>> q_tables_;
    std::vector<policies::EpsilonGreedyPolicy> policies_;
    std::vector<QLearningAgent> agents_;
};

inline IndependentQLearning::IndependentQLearning(uint_t n_agents, uint_t n_states,
                                                  uint_t n_actions,
                                                  const IndependentQLearningConfig &config)
    : config_(config)
{
    if (n_agents == 0 || n_states == 0 || n_actions == 0)
    {
        throw std::invalid_argument("IndependentQLearning: needs agents, states and actions");
    }
    q_tables_.assign(n_agents, DynMat<real_t>::Zero(n_states, n_actions));
    policies_.reserve(n_agents);
    for (uint_t a = 0; a < n_agents; ++a)
    {
        // the agents must not explore in step, nor share a generator
        policies_.emplace_back(config.eps_max,
                               bitsim::utils::concurrency::task_seed(config.seed, a),
                               policies::EpsilonDecayOption::EXPONENTIAL, config.eps_min,
                               config.eps_max, config.eps_decay);
    agents_.reserve(n_agents);
    for (uint_t a = 0; a < n_agents; ++a)
    {
        agents_.emplace_back(a, n_states, n_actions, config);
    }
}

template <typename TimeStepType>
IndependentQLearning::joint_action_type IndependentQLearning::act(const TimeStepType &step)
{
    joint_action_type actions(n_agents(), 0);
    for (uint_t a = 0; a < n_agents(); ++a)
    {
        if (!step.done(a))
        {
            actions[a] = policies_[a](q_tables_[a].row(step.observation(a)));
        }
        actions[a] = agents_[a].act(step);
    }
    return actions;
}

template <typename TimeStepType>
IndependentQLearning::joint_action_type
IndependentQLearning::act_greedy(const TimeStepType &step) const
{
    joint_action_type actions(n_agents(), 0);
    for (uint_t a = 0; a < n_agents(); ++a)
    {
        actions[a] = agents_[a].act_greedy(step);
        if (!step.done(a))
        {
            Eigen::Index best = 0;
            q_tables_[a].row(step.observation(a)).maxCoeff(&best);
            actions[a] = static_cast<uint_t>(best);
        }
    }
    return actions;
}

template <typename TimeStepType>
void IndependentQLearning::update(const TimeStepType &step, const joint_action_type &actions,
                                  const TimeStepType &next)
{
    for (uint_t a = 0; a < n_agents(); ++a)
    {
        agents_[a].update(step, actions[a], next);
        if (step.done(a))
        {
            continue;
        }
        auto &q = q_tables_[a];
        const uint_t o = step.observation(a);
        const uint_t o_next = next.observation(a);
        const real_t bootstrap = next.done(a) ? 0.0 : config_.gamma * q.row(o_next).maxCoeff();
        const real_t target = next.reward(a) + bootstrap;
        q(o, actions[a]) += config_.eta * (target - q(o, actions[a]));
    }
}

inline void IndependentQLearning::on_episode_end(uint_t episode)
{
    for (auto &agent : agents_)
    {
        agent.on_episode_end(episode);
    for (auto &policy : policies_)
    {
        policy.on_episode(episode + 1);
    }
}

inline void IndependentQLearning::save(const std::string &prefix) const
{
    for (uint_t a = 0; a < n_agents(); ++a)
    {
        bitsim::utils::io::CSVWriter writer(prefix + std::to_string(a) + ".csv", ',');
        writer.open();
        const auto &q = agents_[a].q_table();
        const auto &q = q_tables_[a];
        for (Eigen::Index s = 0; s < q.rows(); ++s)
        {
            std::vector<real_t> row(q.cols());
            for (Eigen::Index c = 0; c < q.cols(); ++c)
            {
                row[c] = q(s, c);
            }
            writer.write_row(row);
        }
        writer.close();
    }
}

} // namespace rl::multi_agent
} // namespace bitrl

#endif // INDEPENDENT_Q_LEARNING_H
