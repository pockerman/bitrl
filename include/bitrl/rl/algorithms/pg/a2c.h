#ifndef A2C_H
#define A2C_H

///
/// Implements synchronous  advantage-actor critic, A2C, algorithm
/// Currently the implementation of this class assumes that
/// PyTorch is used to model the deep networks
///
///

#include "bitrl/base/bitrl_config.h"

#ifdef USE_PYTORCH

#include "bitrl/base/bitrl_types.h"
#include "bitrl/rl/algorithms/pg/a2c_config.h"
#include "bitrl/rl/algorithms/pg/a2c_monitor.h"
#include "bitrl/rl/algorithms/rl_algorithm_base.h"
#include "bitrl/rl/algorithms/utils.h"
#include "bitrl/rl/episode_info.h"
#include "bitrl/utils/torch_adaptor.h"
//#include "bitrl/utils/experience_buffer.h"

#include <torch/torch.h>

#ifdef BITRL_DEBUG
#include <cassert>
#endif

#include <chrono>
#include <memory>
#include <tuple>

namespace bitrl
{
namespace rl::algos::pg
{

///
/// \brief A2C solver assuming separate networks for the actor and
/// the critic.
/// Similar to the Reinforce algorithm implementation,
/// the PolicyType should expose an act method with the following signature
///
///  std::tuple[ActioType, torch::Tensor> act(StateType)
///
///  In addition the CriticType should expose an evaluate method
///  with the following signature
///
///  torch::Tensor evaluate(StateType)
///
template <typename EnvType, typename PolicyType, typename CriticType>
class A2CSolver final : public RLSolverBase<EnvType>
{
  public:
    ///
    /// \brief The environment type
    ///
    typedef EnvType env_type;

    ///
    /// \brief The  policy or action type
    ///
    typedef PolicyType policy_type;

    ///
    /// \brief The critic type
    ///
    typedef CriticType critic_type;

    typedef typename env_type::state_type state_type;
    typedef typename env_type::action_type action_type;

    typedef
        typename A2CMonitor<action_type, state_type>::experience_buffer_type experience_buffer_type;

    ///
    /// \brief A2C
    /// \param config
    /// \param policy
    ///
    A2CSolver(const A2CConfig &config, policy_type &policy, critic_type &critic,
              std::unique_ptr<torch::optim::Optimizer> &policy_optimizer,
              std::unique_ptr<torch::optim::Optimizer> &critic_optimizer);

    ///
    /// \brief actions_before_training_begins. Execute any actions the
    /// algorithm needs before starting the iterations
    ///
    virtual void actions_before_training_begins(env_type &);

    ///
    /// \brief actions_after_training_ends. Actions to execute after
    /// the training iterations have finisehd
    ///
    virtual void actions_after_training_ends(env_type &) override final {}

    ///
    /// \brief actions_before_training_episode
    ///
    virtual void actions_before_episode_begins(env_type &, uint_t /*episode_idx*/) override final {}

    ///
    /// \brief actions_after_training_episode
    ///
    virtual void actions_after_episode_ends(env_type &, uint_t /*episode_idx*/,
                                            const EpisodeInfo &) override final
    {
    }

    ///
    /// \brief on_episode Do one on_episode of the algorithm
    ///
    virtual EpisodeInfo on_training_episode(env_type &, uint_t /*episode_idx*/);

    ///
    /// \brief set_train_mode for both the Actor and the Critic
    ///
    void set_train_mode() noexcept;

    ///
    /// \brief set_evaluation_mode for both the Actor and the Critic
    ///
    void set_evaluation_mode() noexcept;

    ///
    /// \brief Read-write access to the monitor object
    ///
    A2CMonitor<action_type, state_type> &get_monitor() { return monitor_; }

  private:
    ///
    /// \brief config_
    ///
    A2CConfig config_;

    ///
    /// \brief policy_
    ///
    policy_type &policy_;

    ///
    /// \brief The action network
    ///
    critic_type &critic_;

    ///
    /// \brief Helper class to monitor the algorithm
    ///
    A2CMonitor<action_type, state_type> monitor_;

    ///
    /// \brief The policy_ optimzer
    ///
    std::unique_ptr<torch::optim::Optimizer> policy_optimizer_;

    ///
    /// \brief The optimizer for the critic network
    ///
    std::unique_ptr<torch::optim::Optimizer> critic_optimizer_;

    ///
    /// \brief build the episode buffer that will be used
    /// for updating the networks
    ///
    uint_t create_episode_batch_(env_type &, uint_t /*episode_idx*/,
                                 experience_buffer_type &buffer);

    std::tuple<real_t, real_t> train_with_batch_(experience_buffer_type &buffer);
};

template <typename EnvType, typename PolicyType, typename CriticType>
A2CSolver<EnvType, PolicyType, CriticType>::A2CSolver(
    const A2CConfig &config, policy_type &policy, critic_type &critic,
    std::unique_ptr<torch::optim::Optimizer> &policy_optimizer,
    std::unique_ptr<torch::optim::Optimizer> &critic_optimizer)
    : config_(config), policy_(policy), critic_(critic), monitor_(),
      policy_optimizer_(std::move(policy_optimizer)), critic_optimizer_(std::move(critic_optimizer))
{
}

template <typename EnvType, typename PolicyType, typename CriticType>
void A2CSolver<EnvType, PolicyType, CriticType>::set_train_mode() noexcept
{
    policy_->train();
    critic_->train();
}

template <typename EnvType, typename PolicyType, typename CriticType>
void A2CSolver<EnvType, PolicyType, CriticType>::set_evaluation_mode() noexcept
{
    policy_->eval();
    critic_->eval();
}

template <typename EnvType, typename PolicyType, typename CriticType>
void A2CSolver<EnvType, PolicyType, CriticType>::actions_before_training_begins(env_type & /*env*/)
{

    monitor_.reset();
    monitor_.policy_loss_values.reserve(config_.n_episodes);
    monitor_.critic_loss_values.reserve(config_.n_episodes);
    monitor_.rewards.reserve(config_.n_episodes);
    monitor_.episode_duration.reserve(config_.n_episodes);
    set_train_mode();
}

template <typename EnvType, typename PolicyType, typename CriticType>
EpisodeInfo A2CSolver<EnvType, PolicyType, CriticType>::on_training_episode(env_type &env,
                                                                            uint_t episode_idx)
{

    auto start = std::chrono::steady_clock::now();

    // the buffer to use
    experience_buffer_type buffer(config_.max_itrs_per_episode);

    // collect the buffer
    auto eps_itrs = create_episode_batch_(env, episode_idx, buffer);

    // train the networks with from the
    // collected buffer
    auto [episode_reward, total_episode_loss] = train_with_batch_(buffer);

    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<real_t> elapsed_seconds = end - start;

    monitor_.episode_duration.push_back(eps_itrs);

    EpisodeInfo info;
    info.episode_index = episode_idx;
    info.episode_reward = episode_reward;
    info.episode_iterations = eps_itrs;
    info.total_time = elapsed_seconds;
    return info;
}

template <typename EnvType, typename PolicyType, typename CriticType>
uint_t A2CSolver<EnvType, PolicyType, CriticType>::create_episode_batch_(
    env_type &env, uint_t /*episode_idx*/, experience_buffer_type &buffer)
{
    /// we want to push into the monitor
    /// experience tuples
    typedef
        typename A2CMonitor<action_type, state_type>::experience_tuple_type experience_tuple_type;

    // reset the environment
    //  for every episode reset the environment
    auto old_timestep = env.reset();

    // loop over the iterations
    uint_t itrs = 0;
    for (; itrs < config_.max_itrs_per_episode; ++itrs)
    {

        auto [action, log_prob] = policy_->act(old_timestep.observation());
        auto values = critic_->evaluate(old_timestep.observation());

        // step into the environment
        auto next_time_step = env.step(action);

        auto next_state = next_time_step.observation();
        auto reward = next_time_step.reward();

        experience_tuple_type exp = {old_timestep.observation(), action,   reward,
                                     next_time_step.done(),      log_prob, values};

        // put the observation into the buffer
        buffer.append(exp);

        if (next_time_step.done())
        {
            break;
        }

        old_timestep = next_time_step;
    }

    return itrs + 1;
}

template <typename EnvType, typename PolicyType, typename CriticType>
std::tuple<real_t, real_t>
A2CSolver<EnvType, PolicyType, CriticType>::train_with_batch_(experience_buffer_type &buffer)
{

    // because of the way we treat the values
    // we loose the requires_grad so we need to set it
    using namespace bitrl::utils::pytorch;

    typedef
        typename A2CMonitor<action_type, state_type>::experience_tuple_type experience_tuple_type;
    typedef std::vector<experience_tuple_type> batch_type;

    // the batch for this episode
    auto batch = buffer.template get<batch_type>();
    auto rewards_batch = monitor_.template get<real_t, 2>(batch);
    auto values_batch = monitor_.template get<torch_tensor_t, 5>(batch);
    auto logprobs_batch = monitor_.template get<torch_tensor_t, 4>(batch);

    // compute the discounted rewards for this batch
    auto discounted_returns =
        bitrl::rl::algos::calculate_step_discounted_return(rewards_batch, config_.gamma);

    auto torch_rewards_batch =
        TorchAdaptor::to_torch(discounted_returns, config_.device_type, false);

    auto torch_values_batch = TorchAdaptor::stack(values_batch, config_.device_type);

    auto torch_logprobs_batch = TorchAdaptor::stack(logprobs_batch, config_.device_type);

    // form the advantage
    auto advantage = torch_rewards_batch - torch_values_batch;

    // take the mean because we collect batches
    auto actor_loss = -(torch_logprobs_batch * advantage.detach()).mean();
    auto critic_loss = advantage.pow(2).mean();

    if (config_.clip_policy_grad)
    {

        // clip the grad if needed
        torch::nn::utils::clip_grad_norm_(policy_->parameters(), config_.max_grad_norm_policy);
    }

    if (config_.clip_critic_grad)
    {
        torch::nn::utils::clip_grad_norm_(critic_->parameters(), config_.max_grad_norm_critic);
    }

    // Backward pass and optimize
    policy_optimizer_->zero_grad();
    critic_optimizer_->zero_grad();

    actor_loss.backward();
    critic_loss.backward();

    policy_optimizer_->step();
    critic_optimizer_->step();

    auto total_episode_policy_loss = actor_loss.item().template to<real_t>();
    auto total_episode_critic_loss = critic_loss.item().template to<real_t>();

    // compute the undiscounted reward as the reward
    // for this episode
    auto R = bitrl::maths::sum(rewards_batch);

    monitor_.policy_loss_values.push_back(total_episode_policy_loss);
    monitor_.critic_loss_values.push_back(total_episode_critic_loss);
    monitor_.rewards.push_back(R);

    return std::make_tuple(R, total_episode_policy_loss + total_episode_critic_loss);
}

} // namespace rl::algos::pg
} // namespace bitrl
#endif
#endif // A2C_H
