#ifndef EXPECTED_SARSA_H
#define EXPECTED_SARSA_H

#include "bitrl/base/bitrl_types.h"
#include "bitrl/maths/matrix_utilities.h"
#include "bitrl/rl/algorithms/td/td_algo_base.h"
#include "bitrl/rl/episode_info.h"
#include "bitrl/rl/policies/max_tabular_policy.h"
#include "bitrl/rl/worlds/envs_concepts.h"
#include "bitsim/bitsim_consts.h"
#include "bitsim/utils/io/csv_file_writer.h"

#include <chrono>
#include <concepts>
#include <string>
#include <vector>

namespace bitrl
{
namespace rl::algos::td
{

///
/// \brief The configuration of ExpectedSarsaSolver
///
struct ExpectedSarsaConfig
{
    ///
    /// \brief Not used; the trainer decides how many episodes to run
    ///
    uint_t n_episodes{100};

    ///
    /// \brief Not used; the trainer decides when training stops
    ///
    real_t tolerance{1.0e-8};

    ///
    /// \brief The discount factor
    ///
    real_t gamma{1.0};

    ///
    /// \brief The learning rate
    ///
    real_t eta{0.1};

    ///
    /// \brief The most steps an episode can take
    ///
    uint_t max_num_iterations_per_episode{100};

    ///
    /// \brief Where to save the table when training ends; nothing is saved if
    /// it is not set
    ///
    std::string path{bitsim::consts::INVALID_STR};
};

///
/// \brief What ExpectedSarsaSolver needs from its policy: it picks an action
/// from the table and a state, it is epsilon-greedy and tells its epsilon,
/// and it can adjust itself after every episode. EpsilonGreedyPolicy is one
///
template <typename PolicyType>
concept epsilon_greedy_policy_concept = requires(PolicyType policy, const PolicyType cpolicy,
                                                 const DynMat<real_t> &q, uint_t state)
{
    {
        policy(q, state)
        } -> std::convertible_to<uint_t>;
    {
        cpolicy.eps_value()
        } -> std::convertible_to<real_t>;
    policy.on_episode(state);
};

///
/// \brief Tabular expected SARSA, see Sutton and Barto, Reinforcement
/// Learning: An Introduction, section 6.6.
///
/// SARSA moves Q(s, a) towards r + gamma Q(s', a'), with a' the action its
/// policy takes next. Expected SARSA uses the value the policy expects at s'
/// instead,
///
///     r + gamma sum_a pi(a | s') Q(s', a),
///
/// which does not depend on which action it happens to take, so its updates
/// vary less. For an epsilon-greedy policy over n actions, pi(a | s') is
/// epsilon / n for every action, plus 1 - epsilon for the greedy one. After a
/// step that ends the episode the target is the reward alone.
///
template <envs::discrete_world_concept EnvType, epsilon_greedy_policy_concept PolicyType>
class ExpectedSarsaSolver final : public TDAlgoBase<EnvType>
{
  public:
    typedef typename TDAlgoBase<EnvType>::env_type env_type;
    typedef typename TDAlgoBase<EnvType>::action_type action_type;
    typedef typename TDAlgoBase<EnvType>::state_type state_type;
    typedef PolicyType policy_type;

    ///
    /// \brief Constructor
    ///
    ExpectedSarsaSolver(const ExpectedSarsaConfig &config, const PolicyType &policy);

    ///
    /// \brief Sets every entry of the table to zero
    ///
    virtual void actions_before_training_begins(env_type &);

    ///
    /// \brief Saves the table if the configuration gives a path
    ///
    virtual void actions_after_training_ends(env_type &);

    virtual void actions_before_episode_begins(env_type &, uint_t /*episode_idx*/) {}

    ///
    /// \brief Lets the policy adjust itself, for example decay its epsilon
    ///
    virtual void actions_after_episode_ends(env_type &, uint_t episode_idx,
                                            const EpisodeInfo & /*einfo*/)
    {
        policy_.on_episode(episode_idx);
    }

    ///
    /// \brief Plays one episode and updates the table after every step
    ///
    virtual EpisodeInfo on_training_episode(env_type &, uint_t episode_idx);

    ///
    /// \brief Writes the table to a CSV file, one row per state
    ///
    void save(const std::string &filename) const;

    ///
    /// \brief The greedy policy of the table
    ///
    bitrl::rl::policies::MaxTabularPolicy build_policy() const;

    ///
    /// \brief The value of every state and action
    ///
    const DynMat<real_t> &q_table() const noexcept { return q_table_; }

    ///
    /// \brief The value the policy expects at the given state: every action
    /// weighed with the probability the policy takes it
    ///
    real_t expected_value(const state_type &state) const;

  private:
    ExpectedSarsaConfig config_;
    policy_type policy_;
    DynMat<real_t> q_table_;
};

template <envs::discrete_world_concept EnvType, epsilon_greedy_policy_concept PolicyType>
ExpectedSarsaSolver<EnvType, PolicyType>::ExpectedSarsaSolver(const ExpectedSarsaConfig &config,
                                                              const PolicyType &policy)
    : TDAlgoBase<EnvType>(), config_(config), policy_(policy)
{
}

template <envs::discrete_world_concept EnvType, epsilon_greedy_policy_concept PolicyType>
void ExpectedSarsaSolver<EnvType, PolicyType>::actions_before_training_begins(env_type &env)
{
    q_table_ = DynMat<real_t>::Zero(env.n_states(), env.n_actions());
}

template <envs::discrete_world_concept EnvType, epsilon_greedy_policy_concept PolicyType>
void ExpectedSarsaSolver<EnvType, PolicyType>::actions_after_training_ends(env_type &)
{
    if (config_.path != bitsim::consts::INVALID_STR)
    {
        save(config_.path);
    }
}

template <envs::discrete_world_concept EnvType, epsilon_greedy_policy_concept PolicyType>
real_t ExpectedSarsaSolver<EnvType, PolicyType>::expected_value(const state_type &state) const
{
    const DynVec<real_t> values = maths::get_row(q_table_, state);
    const auto n_actions = static_cast<real_t>(values.size());
    const real_t eps = policy_.eps_value();

    // the greedy action, picked the way the policy picks it
    const auto greedy = bitrl::rl::policies::MaxTabularPolicy::get_action(values);
    return eps * values.sum() / n_actions + (1.0 - eps) * values[greedy];
}

template <envs::discrete_world_concept EnvType, epsilon_greedy_policy_concept PolicyType>
EpisodeInfo ExpectedSarsaSolver<EnvType, PolicyType>::on_training_episode(env_type &env,
                                                                          uint_t episode_idx)
{
    auto start = std::chrono::steady_clock::now();
    EpisodeInfo info;

    auto episode_score = 0.0;
    auto state = env.reset().observation();

    uint_t itr = 0;
    for (; itr < config_.max_num_iterations_per_episode; ++itr)
    {
        const auto action = policy_(q_table_, state);
        auto time_step = env.step(action);
        const auto next_state = time_step.observation();
        const auto reward = time_step.reward();
        const auto done = time_step.done();
        episode_score += reward;

        // nothing follows the step that ends the episode
        const auto target = done ? reward : reward + config_.gamma * expected_value(next_state);
        q_table_(state, action) += config_.eta * (target - q_table_(state, action));

        if (done)
        {
            break;
        }
        state = next_state;
    }

    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<real_t> elapsed_seconds = end - start;

    info.episode_index = episode_idx;
    info.episode_reward = episode_score;
    info.episode_iterations = itr;
    info.total_time = elapsed_seconds;
    return info;
}

template <envs::discrete_world_concept EnvType, epsilon_greedy_policy_concept PolicyType>
void ExpectedSarsaSolver<EnvType, PolicyType>::save(const std::string &filename) const
{
    bitsim::utils::io::CSVWriter file_writer(filename, ',');
    file_writer.open();

    std::vector<std::string> col_names(1 + q_table_.cols());
    col_names[0] = "state_index";
    for (uint_t i = 0; i < static_cast<uint_t>(q_table_.cols()); ++i)
    {
        col_names[i + 1] = "action_" + std::to_string(i);
    }
    file_writer.write_column_names(col_names);

    // a tuple with a DynVec in it would write the values in one column, so
    // every row is a vector of numbers
    std::vector<real_t> row(1 + q_table_.cols());
    for (uint_t s = 0; s < static_cast<uint_t>(q_table_.rows()); ++s)
    {
        row[0] = static_cast<real_t>(s);
        for (uint_t a = 0; a < static_cast<uint_t>(q_table_.cols()); ++a)
        {
            row[1 + a] = q_table_(s, a);
        }
        file_writer.write_row(row);
    }
}

template <envs::discrete_world_concept EnvType, epsilon_greedy_policy_concept PolicyType>
bitrl::rl::policies::MaxTabularPolicy ExpectedSarsaSolver<EnvType, PolicyType>::build_policy() const
{
    bitrl::rl::policies::MaxTabularPolicy policy;
    bitrl::rl::policies::MaxTabularPolicyBuilder builder;
    builder.build_from_state_action_function(q_table_, policy);
    return policy;
}

} // namespace rl::algos::td
} // namespace bitrl

#endif // EXPECTED_SARSA_H
