#ifndef DOUBLE_Q_LEARNING_H
#define DOUBLE_Q_LEARNING_H

///
/// Implementation of tabular double q-learning algorithm
///
///

#include "cubeai/base/cubeai_types.h"
#include "cubeai/maths/matrix_utilities.h"
#include "cubeai/rl/algorithms/td/td_algo_base.h"
#include "cubeai/rl/episode_info.h"
#include "cubeai/rl/rl_mixins.h"
#include "cubeai/rl/worlds/envs_concepts.h"

#include "bitsim/bitsim_consts.h"
#include "bitsim/utils/io/csv_file_writer.h"

#include <chrono>
#include <random>

namespace bitrl
{
namespace rl::algos::td
{

struct DoubleQLearningConfig
{

    std::string path{bitsim::consts::INVALID_STR};
    real_t tolerance;
    real_t gamma;
    real_t eta;
    uint_t max_num_iterations_per_episode;
    uint_t n_episodes;
    uint_t seed{42};
};

///
/// \brief The class DoubleQLearning. Simple tabular implemtation
/// of double q-learning algorithm.
///
template <envs::discrete_world_concept EnvTp, typename ActionSelector>
class DoubleQLearning final : public TDAlgoBase<EnvTp>,
                              protected with_double_q_table_mixin<DynMat<real_t>>,
                              protected with_double_q_table_max_action_mixin
{
  public:
    ///
    /// \brief env_t
    ///
    typedef typename TDAlgoBase<EnvTp>::env_type env_type;

    ///
    /// \brief action_t
    ///
    typedef typename TDAlgoBase<EnvTp>::action_type action_type;

    ///
    /// \brief state_t
    ///
    typedef typename TDAlgoBase<EnvTp>::state_type state_type;

    ///
    /// \brief action_selector_t
    ///
    typedef ActionSelector action_selector_type;

    ///
    /// \brief Constructor
    ///
    DoubleQLearning(const DoubleQLearningConfig config, const ActionSelector &selector);

    ///
    /// \brief actions_before_training_begins. Execute any actions the
    /// algorithm needs before starting the iterations
    ///
    virtual void actions_before_training_begins(env_type &);

    ///
    /// \brief actions_after_training_ends. Actions to execute after
    /// the training iterations have finisehd
    ///
    virtual void actions_after_training_ends(env_type &);

    ///
    /// \brief actions_before_training_episode
    ///
    virtual void actions_before_episode_begins(env_type &, uint_t /*episode_idx*/) {}

    ///
    /// \brief actions_after_training_episode
    ///
    virtual void actions_after_episode_ends(env_type &, uint_t episode_idx,
                                            const EpisodeInfo & /*einfo*/)
    {
        action_selector_.adjust_on_episode(episode_idx);
    }

    ///
    /// \brief on_episode Do one on_episode of the algorithm
    ///
    virtual EpisodeInfo on_training_episode(env_type &, uint_t episode_idx);

    ///
    ///
    ///
    void save(std::string filename) const;

  private:
    DoubleQLearningConfig config_;

    ///
    /// \brief action_selector_. This is typically the policy
    /// we use to select actions. e.g. epsilon-greedy or softmax
    ///
    action_selector_type action_selector_;

    ///
    /// \brief update_q_table_
    /// \param action
    ///
    void update_q_table_(const action_type &action, const state_type &cstate,
                         const state_type &next_state, real_t reward);
};

template <envs::discrete_world_concept EnvTp, typename ActionSelector>
DoubleQLearning<EnvTp, ActionSelector>::DoubleQLearning(const DoubleQLearningConfig config,
                                                        const ActionSelector &selector)
    : TDAlgoBase<EnvTp>(), with_double_q_table_mixin<DynMat<real_t>>(), config_(config),
      action_selector_(selector)
{
}

template <envs::discrete_world_concept EnvTp, typename ActionSelector>
void DoubleQLearning<EnvTp, ActionSelector>::actions_before_training_begins(env_type &env)
{
    this->with_double_q_table_mixin<DynMat<real_t>>::initialize(env.n_states(), env.n_actions(),
                                                                0.0);
}

template <envs::discrete_world_concept EnvTp, typename ActionSelector>
EpisodeInfo DoubleQLearning<EnvTp, ActionSelector>::on_training_episode(env_type &env,
                                                                        uint_t episode_idx)
{

    auto start = std::chrono::steady_clock::now();
    EpisodeInfo info;

    // total score for the episode
    auto episode_score = 0.0;

    auto state = env.reset().observation();

    uint_t itr = 0;
    for (; itr < config_.max_num_iterations_per_episode; ++itr)
    {

        // select an action
        auto action =
            action_selector_(this->with_double_q_table_mixin<DynMat<real_t>>::q_table_1,
                             this->with_double_q_table_mixin<DynMat<real_t>>::q_table_2, state);

        // Take an action on the environment
        auto step_type_result = env.step(action);

        auto next_state = step_type_result.observation();
        auto reward = step_type_result.reward();
        auto done = step_type_result.done();

        // accumulate score
        episode_score += reward;

        // update the table
        update_q_table_(action, state, next_state, reward);
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
                                                             real_t reward)
{

    // flip a coin 50% of the time we update Q1
    // whilst 50% of the time Q2
    std::mt19937 gen(config_.seed); // rd());

    // generate a number in [0, 1]
    std::uniform_real_distribution<> real_dist_(0.0, 1.0);

    // update Q1
    if (real_dist_(gen) <= 0.5)
    {

        // the current qvalue
        auto q_current =
            this->with_double_q_table_mixin<DynMat<real_t>>::template get<1>(cstate, action);
        auto Qsa_next = 0.0;

        // if(this->env_ref_().is_valid_state(next_state)){
        auto max_act = this->with_double_q_table_max_action_mixin::max_action(
            this->with_double_q_table_mixin<DynMat<real_t>>::q_table_1, next_state,
            this->env_ref_().n_actions());

        // value of next state
        Qsa_next =
            this->with_double_q_table_mixin<DynMat<real_t>>::template get<2>(next_state, max_act);
        //}

        // construct TD target
        auto target = reward + (config_.gamma * Qsa_next);

        // get updated value
        auto new_value = q_current + (config_.eta * (target - q_current));
        this->with_double_q_table_mixin<DynMat<real_t>>::template set<1>(cstate, action, new_value);
    }
    else
    {

        // the current qvalue
        auto q_current =
            this->with_double_q_table_mixin<DynMat<real_t>>::template get<2>(cstate, action);
        auto Qsa_next = 0.0;

        auto max_act = this->with_double_q_table_max_action_mixin::max_action(
            this->with_double_q_table_mixin<DynMat<real_t>>::q_table_2, next_state,
            this->env_ref_().n_actions());

        // value of next state
        Qsa_next =
            this->with_double_q_table_mixin<DynMat<real_t>>::template get<1>(next_state, max_act);

        // construct TD target
        auto target = reward + (config_.gamma * Qsa_next);

        // get updated value
        auto new_value = q_current + (config_.eta * (target - q_current));
        this->with_double_q_table_mixin<DynMat<real_t>>::template set<2>(cstate, action, new_value);
    }
}

template <envs::discrete_world_concept EnvTp, typename ActionSelector>
void DoubleQLearning<EnvTp, ActionSelector>::save(std::string filename) const
{

    rlenvscpp::utils::io::CSVWriter file_writer(filename, ',', true);
    std::vector<std::string> col_names(
        1 + this->with_double_q_table_mixin<DynMat<real_t>>::q_table_1.columns());
    col_names[0] = "state_index";

    for (uint_t i = 0; i < this->with_double_q_table_mixin<DynMat<real_t>>::q_table_1.columns();
         ++i)
    {
        col_names[i + 1] = "action_" + std::to_string(i);
    }

    file_writer.write_column_names(col_names);

    for (uint_t s = 0; s < this->with_double_q_table_mixin<DynMat<real_t>>::q_table_1.rows(); ++s)
    {

        auto actions =
            maths::get_row(this->with_double_q_table_mixin<DynMat<real_t>>::q_table_1, s);
        file_writer.write_row(std::make_tuple(s, actions));

        actions = maths::get_row(this->with_double_q_table_mixin<DynMat<real_t>>::q_table_2, s);
        file_writer.write_row(std::make_tuple(s, actions));
    }
}

} // namespace rl::algos::td
} // namespace bitrl

#endif // DOUBLE_Q_LEARNING_H
