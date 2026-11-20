#ifndef DOUBLE_Q_LEARNING_H
#define DOUBLE_Q_LEARNING_H

///
/// Tabular double Q-learning
///

#include "bitrl/base/bitrl_types.h"
#include "bitrl/maths/matrix_utilities.h"
#include "bitrl/rl/algorithms/td/td_algo_base.h"
#include "bitrl/rl/episode_info.h"
#include "bitrl/rl/rl_mixins.h"
#include "bitrl/rl/worlds/envs_concepts.h"
#include "bitsim/bitsim_consts.h"
#include "bitsim/utils/io/csv_file_writer.h"

#include <chrono>
#include <random>
#include <string>
#include <tuple>
#include <vector>

namespace bitrl
{
namespace rl::algos::td
{

///
/// \brief The configuration of DoubleQLearning
///
struct DoubleQLearningConfig
{
    ///
    /// \brief Where to save the two tables when training ends; nothing is
    /// saved if it is not set
    ///
    std::string path{bitsim::consts::INVALID_STR};

    ///
    /// \brief Not used; the trainer decides when training stops
    ///
    real_t tolerance{1.0e-8};

    ///
    /// \brief The discount factor
    ///
    real_t gamma{0.99};

    ///
    /// \brief The learning rate
    ///
    real_t eta{0.1};

    ///
    /// \brief The most steps an episode can take
    ///
    uint_t max_num_iterations_per_episode{100};

    ///
    /// \brief Not used; the trainer decides how many episodes to run
    ///
    uint_t n_episodes{100};

    ///
    /// \brief The seed of the coin that picks which table a step updates
    ///
    uint_t seed{42};
};

///
/// \brief The class DoubleQLearning. Tabular double Q-learning, see
/// van Hasselt, Double Q-learning, NIPS 2010.
///
/// Q-learning takes the value of the next state as the largest value in the
/// same table it updates, which overestimates it. Double Q-learning keeps two
/// tables. Every step a coin picks one of them to update; the action that is
/// best in that table is valued with the other one. Actions are chosen with
/// the sum of the two tables.
///
/// ActionSelector is called as selector(q_table_1, q_table_2, state) and
/// returns an action, for example EpsilonDoubleQTableGreedyPolicy. It must
/// also have adjust_on_episode(episode_idx).
///
template <envs::discrete_world_concept EnvTp, typename ActionSelector>
class DoubleQLearning final : public TDAlgoBase<EnvTp>,
                              protected with_double_q_table_mixin<DynMat<real_t>>,
                              protected with_double_q_table_max_action_mixin
{
  public:
    typedef typename TDAlgoBase<EnvTp>::env_type env_type;
    typedef typename TDAlgoBase<EnvTp>::action_type action_type;
    typedef typename TDAlgoBase<EnvTp>::state_type state_type;
    typedef ActionSelector action_selector_type;
    typedef with_double_q_table_mixin<DynMat<real_t>> table_mixin_type;

    ///
    /// \brief Constructor
    ///
    DoubleQLearning(const DoubleQLearningConfig config, const ActionSelector &selector);

    ///
    /// \brief Sets both tables to zero
    ///
    virtual void actions_before_training_begins(env_type &);

    ///
    /// \brief Saves the tables if the configuration gives a path
    ///
    virtual void actions_after_training_ends(env_type &);

    virtual void actions_before_episode_begins(env_type &, uint_t /*episode_idx*/) {}

    ///
    /// \brief Lets the action selector adjust itself, for example decay its epsilon
    ///
    virtual void actions_after_episode_ends(env_type &, uint_t episode_idx,
                                            const EpisodeInfo & /*einfo*/)
    {
        action_selector_.adjust_on_episode(episode_idx);
    }

    ///
    /// \brief Plays one episode and updates the tables after every step
    ///
    virtual EpisodeInfo on_training_episode(env_type &, uint_t episode_idx);

    ///
    /// \brief Writes the two tables to a CSV file, one row per state and table
    ///
    void save(std::string filename) const;

    const DynMat<real_t> &q_table_1() const noexcept { return this->table_mixin_type::q_table_1; }
    const DynMat<real_t> &q_table_2() const noexcept { return this->table_mixin_type::q_table_2; }

  private:
    DoubleQLearningConfig config_;
    action_selector_type action_selector_;

    ///
    /// \brief The coin that picks which table a step updates
    ///
    std::mt19937 generator_;

    void update_q_table_(const action_type &action, const state_type &cstate,
                         const state_type &next_state, real_t reward, bool done);
};

template <envs::discrete_world_concept EnvTp, typename ActionSelector>
DoubleQLearning<EnvTp, ActionSelector>::DoubleQLearning(const DoubleQLearningConfig config,
                                                        const ActionSelector &selector)
    : TDAlgoBase<EnvTp>(), table_mixin_type(), config_(config), action_selector_(selector),
      generator_(config.seed)
{
}

template <envs::discrete_world_concept EnvTp, typename ActionSelector>
void DoubleQLearning<EnvTp, ActionSelector>::actions_before_training_begins(env_type &env)
{
    this->table_mixin_type::initialize(env.n_states(), env.n_actions(), 0.0);
}

template <envs::discrete_world_concept EnvTp, typename ActionSelector>
void DoubleQLearning<EnvTp, ActionSelector>::actions_after_training_ends(env_type &)
{
    if (config_.path != bitsim::consts::INVALID_STR)
    {
        save(config_.path);
    }
}

template <envs::discrete_world_concept EnvTp, typename ActionSelector>
EpisodeInfo DoubleQLearning<EnvTp, ActionSelector>::on_training_episode(env_type &env,
                                                                        uint_t episode_idx)
{
    auto start = std::chrono::steady_clock::now();
    EpisodeInfo info;

    auto episode_score = 0.0;
    auto state = env.reset().observation();

    uint_t itr = 0;
    for (; itr < config_.max_num_iterations_per_episode; ++itr)
    {
        auto action = action_selector_(this->table_mixin_type::q_table_1,
                                       this->table_mixin_type::q_table_2, state);

        auto time_step = env.step(action);
        auto next_state = time_step.observation();
        auto reward = time_step.reward();
        auto done = time_step.done();

        episode_score += reward;
        update_q_table_(action, state, next_state, reward, done);
        state = next_state;

        if (done)
        {
            break;
        }
    }

    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<real_t> elapsed_seconds = end - start;

    info.episode_index = episode_idx;
    info.episode_reward = episode_score;
    info.episode_iterations = itr;
    info.total_time = elapsed_seconds;
    return info;
}

template <envs::discrete_world_concept EnvTp, typename ActionSelector>
void DoubleQLearning<EnvTp, ActionSelector>::update_q_table_(const action_type &action,
                                                             const state_type &cstate,
                                                             const state_type &next_state,
                                                             real_t reward, bool done)
{
    auto &q_table_1 = this->table_mixin_type::q_table_1;
    auto &q_table_2 = this->table_mixin_type::q_table_2;
    const auto n_actions = static_cast<uint_t>(q_table_1.cols());

    // a fair coin picks the table to update; the other one values the
    // action that is best in it
    std::bernoulli_distribution coin(0.5);
    const bool update_first = coin(generator_);
    auto &updated = update_first ? q_table_1 : q_table_2;
    const auto &other = update_first ? q_table_2 : q_table_1;

    // nothing follows a terminal state, so its value is 0
    auto next_value = 0.0;
    if (!done)
    {
        const auto best_action =
            with_double_q_table_max_action_mixin::max_action(updated, next_state, n_actions);
        next_value = other(next_state, best_action);
    }

    const auto target = reward + config_.gamma * next_value;
    const auto current = updated(cstate, action);
    updated(cstate, action) = current + config_.eta * (target - current);
}

template <envs::discrete_world_concept EnvTp, typename ActionSelector>
void DoubleQLearning<EnvTp, ActionSelector>::save(std::string filename) const
{
    const auto &q_table_1 = this->table_mixin_type::q_table_1;
    const auto &q_table_2 = this->table_mixin_type::q_table_2;

    bitsim::utils::io::CSVWriter file_writer(filename, ',');
    file_writer.open();

    // one row per state and table: the state, which of the two tables, and
    // the value of each action in it
    std::vector<std::string> col_names(2 + q_table_1.cols());
    col_names[0] = "state_index";
    col_names[1] = "table";
    for (uint_t i = 0; i < static_cast<uint_t>(q_table_1.cols()); ++i)
    {
        col_names[i + 2] = "action_" + std::to_string(i);
    }
    file_writer.write_column_names(col_names);

    // a tuple with a DynVec in it would write the action values in one
    // column, so every row is a vector of numbers
    const auto n_actions = static_cast<uint_t>(q_table_1.cols());
    std::vector<real_t> row(2 + n_actions);
    for (uint_t s = 0; s < static_cast<uint_t>(q_table_1.rows()); ++s)
    {
        for (uint_t t = 1; t <= 2; ++t)
        {
            const auto &table = t == 1 ? q_table_1 : q_table_2;
            row[0] = static_cast<real_t>(s);
            row[1] = static_cast<real_t>(t);
            for (uint_t a = 0; a < n_actions; ++a)
            {
                row[2 + a] = table(s, a);
            }
            file_writer.write_row(row);
        }
    }
}

} // namespace rl::algos::td
} // namespace bitrl

#endif // DOUBLE_Q_LEARNING_H
