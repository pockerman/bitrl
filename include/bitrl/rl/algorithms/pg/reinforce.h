#ifndef REINFORCE_H
#define REINFORCE_H

///
/// \file simple_reinforce.h
///
/// Implements the REINFORCE algorithm
///

#include "bitrl/base/bitrl_config.h"

#ifdef USE_PYTORCH

#include "bitrl/base/bitrl_types.h"
#include "bitrl/maths/vector_math.h"
#include "bitrl/rl/algorithms/pg/reinforce_config.h"
#include "bitrl/rl/algorithms/pg/reinforce_loss.h"
#include "bitrl/rl/algorithms/pg/reinforce_monitor.h"
#include "bitrl/rl/algorithms/rl_algorithm_base.h"
#include "bitrl/rl/algorithms/utils.h"
#include "bitrl/rl/episode_info.h"
//#include "bitrl/data_structs/experience_buffer.h"
#include "bitrl/base/bitrl_log.h"
#include "bitrl/utils/torch_adaptor.h"

#include <torch/torch.h>

#include <vector>

#include <chrono>
#include <iostream>
#include <iterator>
#include <memory>
#include <numeric>
#include <string>
#include <tuple>

namespace bitrl
{
namespace rl
{
namespace algos
{
namespace pg
{

///
/// \brief The ReinforceSolver class. The ReinforceSolver
/// trains a policy represented by the PolicyTp template parameter
/// on the environment represented by the EnvType parameter.
///
/// The PolicyType should expose an act method with the following
/// signature
///
///  std::tuple[ActioType, torch::Tensor> act(StateType)
///
///
template <typename EnvType, typename PolicyType>
class ReinforceSolver final : public RLSolverBase<EnvType>
{
  public:
    typedef EnvType env_type;
    typedef PolicyType policy_type;

    typedef typename env_type::state_type state_type;
    typedef typename env_type::action_type action_type;
    typedef typename ReinforceMonitor<action_type, state_type>::experience_buffer_type
        experience_buffer_type;

    ///
    /// \brief ReinforceSolver. Constructor
    ///
    ReinforceSolver(ReinforceConfig opts, policy_type &policy,
                    std::unique_ptr<torch::optim::Optimizer> &policy_optimizer);

    ///
    /// \brief actions_before_training_begins. Execute any actions the
    /// algorithm needs before starting the iterations
    ///
    virtual void actions_before_training_begins(env_type &);

    ///
    /// \brief actions_after_training_ends. Actions to execute after
    /// the training iterations have finisehd
    ///
    virtual void actions_after_training_ends(env_type &) {}

    ///
    /// \brief actions_before_training_episode
    ///
    virtual void actions_before_episode_begins(env_type &, uint_t /*episode_idx*/) {}

    ///
    /// \brief actions_after_training_episode
    ///
    virtual void actions_after_episode_ends(env_type &, uint_t /*episode_idx*/,
                                            const EpisodeInfo & /*einfo*/)
    {
    }

    ///
    /// \brief on_episode Do one on_episode of the algorithm
    ///
    virtual EpisodeInfo on_training_episode(env_type &, uint_t /*episode_idx*/);

    ///
    /// \brief Read-write access to the monitor object
    ///
    ReinforceMonitor<action_type, state_type> &get_monitor() { return monitor_; }

  private:
    ///
    /// \brief config_
    ///
    ReinforceConfig config_;

    ///
    /// \brief policy_ptr_
    ///
    policy_type policy_ptr_;

    ///
    /// \brief The policy_ optimzer
    ///
    std::unique_ptr<torch::optim::Optimizer> policy_optimizer_;

    ///
    /// \brief Helper class to monitor the algorithm
    ///
    ReinforceMonitor<action_type, state_type> monitor_;

    ///
    /// \brief create_episode_batch_ Create the episode batch
    /// Simply pushes into the monitor trajectories that follow
    /// the current policy
    ///
    uint_t create_episode_batch_(env_type &env, experience_buffer_type &buffer);

    ///
    /// \brief Train the policy on the buffer using
    /// the whole buffer at once
    ///
    std::tuple<real_t, real_t> train_batch_(experience_buffer_type &buffer);

    ///
    /// \brief Train the policy on the buffer
    /// using one item in the buffer each time
    ///
    std::tuple<real_t, real_t> train_sequential_(experience_buffer_type &buffer);

    ///
    /// \brief Train without baseline
    ///
    std::tuple<real_t, real_t> train_without_baseline_(experience_buffer_type &buffer);

    ///
    /// \brief Train with baseline
    ///
    std::tuple<real_t, real_t> train_with_baseline_(experience_buffer_type &buffer);
};

template <typename EnvType, typename PolicyType>
ReinforceSolver<EnvType, PolicyType>::ReinforceSolver(
    ReinforceConfig config, policy_type &policy,
    std::unique_ptr<torch::optim::Optimizer> &policy_optimizer)
    : RLSolverBase<EnvType>(), config_(config), policy_ptr_(policy),
      policy_optimizer_(std::move(policy_optimizer)), monitor_()

{
}

template <typename EnvType, typename PolicyType>
void ReinforceSolver<EnvType, PolicyType>::actions_before_training_begins(env_type & /*env*/)
{

    monitor_.policy_loss_values.reserve(config_.n_episodes);
    monitor_.rewards.reserve(config_.n_episodes);
    monitor_.episode_duration.reserve(config_.n_episodes);

    // set the policy to train mode
    policy_ptr_->train();
}

template <typename EnvType, typename PolicyType>
uint_t ReinforceSolver<EnvType, PolicyType>::create_episode_batch_(env_type &env,
                                                                   experience_buffer_type &buffer)
{

    /// we want to push into the monitor
    /// experience tuples

    typedef typename ReinforceMonitor<action_type, state_type>::experience_tuple_type
        experience_tuple_type;

    //  for every episode reset the environment
    auto old_timestep = env.reset();

    // iterate over the given number
    // of iterations for the episode and create
    // the trajectory. The trajectory may be less
    // than config_.max_itrs_per_episode

    uint_t itr = 0;
    for (; itr < config_.max_itrs_per_episode; ++itr)
    {

        // from the policy get the action to do based
        // on the seen state.
        auto [action, log_prob] = policy_ptr_->act(old_timestep.observation());

        // execute the selected action on the environment
        auto new_timestep = env.step(action);
        auto reward = new_timestep.reward();

        experience_tuple_type exp = {old_timestep.observation(), action, reward,
                                     new_timestep.done(), log_prob};

        // put the observation into the buffer
        buffer.append(exp);

        if (new_timestep.done())
        {
            break;
        }

        old_timestep = new_timestep;
    }

    // because we start from zero
    return itr + 1;
}

template <typename EnvType, typename PolicyType>
EpisodeInfo ReinforceSolver<EnvType, PolicyType>::on_training_episode(env_type &env,
                                                                      uint_t episode_idx)
{

    // start the time for the episode
    auto start = std::chrono::steady_clock::now();

    // the buffer to use
    experience_buffer_type buffer(config_.max_itrs_per_episode);

    // Accummulate the data i.e. create the
    // batch data we need to train the parameters
    auto itrs = create_episode_batch_(env, buffer);

    EpisodeInfo info;
    if (config_.baseline_type == bitrl::rl::algos::pg::BaselineEnumType::NONE)
    {

        auto [episode_reward, total_episode_loss] = train_without_baseline_(buffer);
        info.episode_reward = episode_reward;
    }
    else
    {

        auto [episode_reward, total_episode_loss] = train_with_baseline_(buffer);
        info.episode_reward = episode_reward;
    }
    monitor_.episode_duration.push_back(itrs);

    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<real_t> elapsed_seconds = end - start;

    // the info class to return for the episode
    info.episode_index = episode_idx;
    info.episode_iterations = itrs;
    info.total_time = elapsed_seconds;
    return info;
}

template <typename EnvType, typename PolicyType>
std::tuple<real_t, real_t>
ReinforceSolver<EnvType, PolicyType>::train_batch_(experience_buffer_type &buffer)
{

    typedef typename ReinforceMonitor<action_type, state_type>::experience_tuple_type
        experience_tuple_type;

    typedef std::vector<experience_tuple_type> batch_type;

    // the batch for this episode
    auto batch = buffer.template get<batch_type>();

    // create the batches
    auto reward_batch = monitor_.template get<real_t, 2>(batch);
    auto log_probs_batch = monitor_.template get<torch_tensor_t, 4>(batch);

    // compute the discounted rewards for this batch
    auto discounted_returns =
        bitrl::rl::algos::calculate_step_discounted_return(reward_batch, config_.gamma);

    if (config_.normalize_rewards)
    {
        discounted_returns = bitrl::maths::normalize_max(discounted_returns);
    }

    std::vector<torch_tensor_t> loss_vals = compute_loss_item(discounted_returns, log_probs_batch);

    auto loss =
        bitrl::utils::pytorch::TorchAdaptor::stack(loss_vals, config_.device_type, true).sum();
    policy_optimizer_->zero_grad();
    loss.backward();
    policy_optimizer_->step();

    auto total_episode_loss = loss.item().to<real_t>();

    // compute the undiscounted reward as the reward
    // for this episode
    auto R = bitrl::maths::sum(reward_batch);
    return std::make_tuple(R, total_episode_loss);
}

template <typename EnvType, typename PolicyType>
std::tuple<real_t, real_t>
ReinforceSolver<EnvType, PolicyType>::train_sequential_(experience_buffer_type &buffer)
{

    typedef typename ReinforceMonitor<action_type, state_type>::experience_tuple_type
        experience_tuple_type;

    typedef std::vector<experience_tuple_type> batch_type;

    // the batch for this episode
    auto batch = buffer.template get<batch_type>();

    // create the batches
    auto reward_batch = monitor_.template get<real_t, 2>(batch);
    auto log_probs_batch = monitor_.template get<torch_tensor_t, 4>(batch);

    // compute the discounted rewards for this batch
    auto discounted_returns =
        bitrl::rl::algos::calculate_step_discounted_return(reward_batch, config_.gamma);

    if (config_.normalize_rewards)
    {
        discounted_returns = bitrl::maths::normalize_max(discounted_returns);
    }

    std::vector<torch_tensor_t> loss_vals = compute_loss_item(discounted_returns, log_probs_batch);

    // auto device =  config_.device_type != DeviceType::CPU ? torch::kCUDA : torch::kCPU;

    auto total_episode_loss = 0.0;
    for (uint_t l = 0; l < loss_vals.size(); ++l)
    {

        auto loss = loss_vals[l];
        policy_optimizer_->zero_grad();
        loss.backward();
        policy_optimizer_->step();

        total_episode_loss += loss.item().to<real_t>();
    }

    auto R = bitrl::maths::sum(reward_batch);
    return std::make_tuple(R, total_episode_loss / loss_vals.size());
}

template <typename EnvType, typename PolicyType>
std::tuple<real_t, real_t>
ReinforceSolver<EnvType, PolicyType>::train_without_baseline_(experience_buffer_type &buffer)
{

    if (config_.train_type == bitrl::utils::TrainEnumType::BATCH)
    {

        auto [episode_reward, total_episode_loss] = train_batch_(buffer);
        monitor_.policy_loss_values.push_back(total_episode_loss);
        monitor_.rewards.push_back(episode_reward);
        return std::make_tuple(episode_reward, total_episode_loss);
    }
    else
    {

        auto [episode_reward, total_episode_loss] = train_sequential_(buffer);
        monitor_.policy_loss_values.push_back(total_episode_loss);
        monitor_.rewards.push_back(episode_reward);
        return std::make_tuple(episode_reward, total_episode_loss);
    }
}

template <typename EnvType, typename PolicyType>
std::tuple<real_t, real_t>
ReinforceSolver<EnvType, PolicyType>::train_with_baseline_(experience_buffer_type &buffer)
{

    typedef typename ReinforceMonitor<action_type, state_type>::experience_tuple_type
        experience_tuple_type;
    typedef std::vector<experience_tuple_type> batch_type;

    // the batch for this episode
    auto batch = buffer.template get<batch_type>();
    auto reward_batch = monitor_.template get<real_t, 2>(batch);

    // compute the discounted rewards for this batch
    auto discounted_returns =
        bitrl::rl::algos::calculate_step_discounted_return(reward_batch, config_.gamma);
    if (config_.baseline_type == BaselineEnumType::CONSTANT)
    {
        discounted_returns =
            compute_baseline_with_constant(discounted_returns, config_.baseline_constant);
    }
    else if (config_.baseline_type == BaselineEnumType::MEAN)
    {
        discounted_returns = compute_baseline_with_mean(discounted_returns);
    }
    else
    {
        discounted_returns = compute_baseline_with_standardization(discounted_returns, config_.eps);
    }

    auto log_probs_batch = monitor_.template get<torch_tensor_t, 4>(batch);
    std::vector<torch_tensor_t> loss_vals = compute_loss_item(discounted_returns, log_probs_batch);

    auto loss =
        bitrl::utils::pytorch::TorchAdaptor::stack(loss_vals, config_.device_type, true).sum();
    policy_optimizer_->zero_grad();
    loss.backward();
    policy_optimizer_->step();

    auto total_episode_loss = loss.item().to<real_t>();

    // compute the undiscounted reward as the reward
    // for this episode
    auto R = bitrl::maths::sum(reward_batch);

    monitor_.policy_loss_values.push_back(total_episode_loss);
    monitor_.rewards.push_back(R);

    return std::make_tuple(R, total_episode_loss);
}

} // namespace pg
} // namespace algos
} // namespace rl
} // namespace bitrl
#endif
#endif // VANILLA_REINFORCE_H
