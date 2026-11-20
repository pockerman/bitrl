// SPDX-FileCopyrightText: 2024 <copyright holder> <email>
// SPDX-License-Identifier: Apache-2.0

#ifndef FIRST_VISIT_MC_H
#define FIRST_VISIT_MC_H

#include "bitrl/base/bitrl_config.h"
#include "bitrl/base/bitrl_types.h"

#include "bitrl/maths/vector_math.h"
#include "bitrl/rl/episode_info.h"
#include "bitsim/bitsim_consts.h"
#include "bitsim/utils/io/csv_file_writer.h"

#ifdef CUBEAI_PRINT_DBG_MSGS
#include <boost/log/trivial.hpp>
#endif

#include <algorithm>
#include <string>
#include <tuple>
#include <vector>

namespace bitrl
{
namespace rl::algos::mc
{

struct FirstVisitMCSolverConfig
{
    real_t gamma{1.0};
    real_t tolerance{1.0e-6};
    real_t init_alpha{0.5};
    real_t min_alpha{0.01};
    real_t alpha_decay_ratio{0.3};
    uint_t max_steps{100};
    uint_t n_episodes{500};
    std::string save_path{bitsim::consts::INVALID_STR};
};

/**
 * @todo write docs
 */
template <typename EnvType, typename TrajectoryGenerator, typename DecayLRSchedule,
          typename DiscountGenerator>
class FirstVisitMCSolver
{
  public:
    /**
     * @brief The environment type
     *
     */
    typedef EnvType env_type;

    /**
     * @brief
     */
    typedef TrajectoryGenerator trajectory_generator_type;

    /**
     * @brief
     */
    typedef DecayLRSchedule decay_lr_schedule_type;

    /**
     * @brief
     **/
    typedef DiscountGenerator discount_generator_type;

    /**
     * @brief The time step type used by the environment
     *
     * */
    typedef typename env_type::time_step_type time_step_type;

    /**
     * @brief Constructor
     *
     **/
    FirstVisitMCSolver(FirstVisitMCSolverConfig solver_config, TrajectoryGenerator &trajectory_gen,
                       DecayLRSchedule &decay_lr_schedule,
                       discount_generator_type &discount_generator);

    ///
    /// \brief actions_before_training_begins. Execute any actions the
    /// algorithm needs before starting the iterations
    ///
    void actions_before_training_begins(env_type &env);

    ///
    /// \brief actions_after_training_ends. Actions to execute after
    /// the training iterations have finisehd
    ///
    void actions_after_training_ends(env_type & /*env*/) {}

    ///
    /// \brief actions_before_training_episode
    ///
    void actions_before_episode_begins(env_type &, uint_t /*episode_idx*/) {}

    ///
    /// \brief actions_after_training_episode
    ///
    void actions_after_episode_ends(env_type &, uint_t /*episode_idx*/,
                                    const EpisodeInfo & /*einfo*/)
    {
    }

    ///
    /// \brief on_episode Do one on_episode of the algorithm
    ///
    EpisodeInfo on_training_episode(env_type &env, uint_t episode_idx);

    /**
     * @brief save the results
     *
     * */
    void save(const std::string &filename) const;

    ///
    /// \brief The value of every state, as the solver has it now
    ///
    DynVec<real_t> value_function() const { return v_; }

  private:
    /**
     * @brief The value function
     *
     **/
    DynVec<real_t> v_;

    /**
     * @brief The configuration of the solver
     *
     **/
    FirstVisitMCSolverConfig config_;

    /**
     * @brief generate the trajector
     *
     */
    TrajectoryGenerator trajectory_gen_;

    /**
     * @brief How to decay the learning rate per episode
     *
     */
    DecayLRSchedule decay_lr_schedule_;

    /**
     * @brief How to create the discounts sequence
     *
     */
    discount_generator_type discount_generator_;
};

template <typename EnvType, typename TrajectoryGenerator, typename DecayLRSchedule,
          typename DiscountGenerator>
FirstVisitMCSolver<EnvType, TrajectoryGenerator, DecayLRSchedule, DiscountGenerator>::
    FirstVisitMCSolver(FirstVisitMCSolverConfig solver_config, TrajectoryGenerator &trajectory_gen,
                       DecayLRSchedule &decay_lr_schedule,
                       discount_generator_type &discount_generator)
    : v_(), config_(solver_config), trajectory_gen_(trajectory_gen),
      decay_lr_schedule_(decay_lr_schedule), discount_generator_(discount_generator)
{
}

template <typename EnvType, typename TrajectoryGenerator, typename DecayLRSchedule,
          typename DiscountGenerator>
void FirstVisitMCSolver<EnvType, TrajectoryGenerator, DecayLRSchedule,
                        DiscountGenerator>::actions_before_training_begins(env_type &env)
{

    v_.resize(env.n_states());
    std::for_each(v_.begin(), v_.end(), [](auto &item) { item = 0.0; });
}

template <typename EnvType, typename TrajectoryGenerator, typename DecayLRSchedule,
          typename DiscountGenerator>
EpisodeInfo FirstVisitMCSolver<EnvType, TrajectoryGenerator, DecayLRSchedule,
                               DiscountGenerator>::on_training_episode(env_type &env,
                                                                       uint_t episode_idx)
{

    // start timing the training on this episode
    auto start = std::chrono::steady_clock::now();

    // generate the trajectory for the environment
    // for this episode
    auto trajectory = trajectory_gen_(env, config_.max_steps);

    const auto trajectory_size = std::distance(trajectory.begin(), trajectory.end());

#ifdef CUBEAI_PRINT_DBG_MSGS
    if (trajectory_size == 0)
    {
        BOOST_LOG_TRIVIAL(warning) << "Trajectory size=" << trajectory_size << std::endl;
    }
#endif

    // accummulate the rewards in an array
    // we need this in order to take the dot product
    // with the discounts
    std::vector<real_t> rewards;
    rewards.reserve(trajectory_size);

    auto time_step_itr = trajectory.begin();
    for (; time_step_itr != trajectory.end(); ++time_step_itr)
    {
        auto time_step = *time_step_itr;
        rewards.push_back(time_step.reward());
    }

    // compute the discounts for the generated trajectory
    auto discounts = discount_generator_(trajectory, config_.max_steps);

    // calculate learning rate
    auto alpha = decay_lr_schedule_(config_.init_alpha, episode_idx);

    std::vector<bool> visited(env.n_states(), false);
    time_step_itr = trajectory.begin();
    for (uint_t count = 0; time_step_itr != trajectory.end(); ++time_step_itr, ++count)
    {

        auto time_step = *time_step_itr;

        if (visited[time_step.observation()])
            continue;

        visited[time_step.observation()] = true;

        // find the steps from the current time_step to the end
        // of the trajectory
        auto n_steps = std::distance(time_step_itr, trajectory.end());

        // calculate the return. First extract up to n_steps
        // from the discounts
        auto trajectory_discounts = bitrl::maths::extract_subvector(discounts, n_steps);
        auto trajectory_rewards = bitrl::maths::extract_subvector(rewards, count, false);
        auto G = bitrl::maths::dot_product(trajectory_discounts, trajectory_rewards);
        auto mc_error = G - v_[time_step.observation()];

        // update the state value
        v_[time_step.observation()] += alpha * mc_error;
    }

    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<real_t> elapsed_seconds = end - start;
    auto episode_info = EpisodeInfo();
    episode_info.episode_index = episode_idx;
    episode_info.total_time = elapsed_seconds;
    episode_info.episode_iterations = std::distance(trajectory.begin(), trajectory.end());
    return episode_info;
}

template <typename EnvType, typename TrajectoryGenerator, typename DecayLRSchedule,
          typename DiscountGenerator>
void FirstVisitMCSolver<EnvType, TrajectoryGenerator, DecayLRSchedule, DiscountGenerator>::save(
    const std::string &filename) const
{

    bitsim::utils::io::CSVWriter file_writer(filename, ',');
    file_writer.open();
    file_writer.write_column_names({"state_index", "value_function"});

    for (uint_t s = 0; s < static_cast<uint_t>(v_.size()); ++s)
    {
        auto row = std::make_tuple(s, v_[s]);
        file_writer.write_row(row);
    }
}

} // namespace rl::algos::mc
} // namespace bitrl
#endif // FIRST_VISIT_MC_H
