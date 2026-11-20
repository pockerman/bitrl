#ifndef PPO_H
#define PPO_H

///
/// Implements Proximal Policy Optimization algorithm
/// Currently the implementation of this class assumes that
/// PyTorch is used to model the deep networks
///
///

#include "bitrl/base/bitrl_config.h"

#ifdef USE_PYTORCH

#include "bitrl/base/bitrl_types.h"
#include "bitrl/rl/algorithms/pg/a2c_monitor.h"
#include "bitrl/rl/algorithms/pg/actor_critic_solver_base.h"
#include "bitrl/rl/algorithms/pg/ppo_config.h"
#include "bitrl/rl/algorithms/utils.h"
#include "bitrl/rl/episode_info.h"
#include "bitrl/utils/torch_adaptor.h"

#include <torch/torch.h>

#ifdef BITRL_DEBUG
#include <boost/log/trivial.hpp>
#include <cassert>
#endif

#include <chrono>

namespace bitrl
{
namespace rl::algos::pg
{

template <typename EnvType, typename PolicyType, typename CriticType>
class PPOSolver final
    : public ACSolverBase<EnvType, PolicyType, CriticType,
                          A2CMonitor<typename EnvType::action_type, typename EnvType::state_type>,
                          PPOConfig>
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

    ///
    /// \brief The state_type
    ///
    typedef typename env_type::state_type state_type;

    ///
    /// \brief  the action type
    ///
    typedef typename env_type::action_type action_type;

    typedef
        typename A2CMonitor<action_type, state_type>::experience_buffer_type experience_buffer_type;

    ///
    /// \brief PPOSolver Constructor
    ///
    PPOSolver(const PPOConfig &config, policy_type &policy, critic_type &critic,
              std::unique_ptr<torch::optim::Optimizer> &policy_optimizer,
              std::unique_ptr<torch::optim::Optimizer> &critic_optimizer);

    ///
    /// \brief on_episode Do one on_episode of the algorithm
    ///
    virtual EpisodeInfo on_training_episode(env_type &, uint_t /*episode_idx*/) override final;

  private:
    std::tuple<real_t, real_t> train_with_batch_(experience_buffer_type &buffer);
};

template <typename EnvType, typename PolicyType, typename CriticType>
PPOSolver<EnvType, PolicyType, CriticType>::PPOSolver(
    const PPOConfig &config, policy_type &policy, critic_type &critic,
    std::unique_ptr<torch::optim::Optimizer> &policy_optimizer,
    std::unique_ptr<torch::optim::Optimizer> &critic_optimizer)
    : ACSolverBase<EnvType, PolicyType, CriticType,
                   A2CMonitor<typename EnvType::action_type, typename EnvType::state_type>,
                   PPOConfig>(config, policy, critic, policy_optimizer, critic_optimizer)
{
}

template <typename EnvType, typename PolicyType, typename CriticType>
EpisodeInfo PPOSolver<EnvType, PolicyType, CriticType>::on_training_episode(env_type &env,
                                                                            uint_t episode_idx)
{
    auto start = std::chrono::steady_clock::now();

    // the buffer to use
    experience_buffer_type buffer(this->config_.max_itrs_per_episode);

    // collect the buffer
    auto eps_itrs = this->create_episode_batch_(env, episode_idx, buffer);

    // train the networks with from the
    // collected buffer
    auto [episode_reward, total_episode_loss] = train_with_batch_(buffer);

    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<real_t> elapsed_seconds = end - start;

    this->monitor_.episode_duration.push_back(eps_itrs);

    EpisodeInfo info;
    info.episode_index = episode_idx;
    info.episode_reward = episode_reward;
    info.episode_iterations = eps_itrs;
    info.total_time = elapsed_seconds;
    return info;
}

template <typename EnvType, typename PolicyType, typename CriticType>
std::tuple<real_t, real_t>
PPOSolver<EnvType, PolicyType, CriticType>::train_with_batch_(experience_buffer_type &buffer)
{

#ifdef BITRL_DEBUG
    BOOST_LOG_TRIVIAL(info) << "PPO: Training with batch...: ";
#endif

    // because of the way we treat the values
    // we loose the requires_grad so we need to set it
    using namespace bitrl::utils::pytorch;
    using namespace bitrl::rl::algos;
    typedef
        typename A2CMonitor<action_type, state_type>::experience_tuple_type experience_tuple_type;
    typedef std::vector<experience_tuple_type> batch_type;

    // the batch for this episode
    auto batch = buffer.template get<batch_type>();
    auto states = this->monitor_.template get<state_type, 0>(batch);
    std::vector<std::vector<float_t>> states_f(states.size());

    for (uint_t i = 0; i < states.size(); ++i)
    {
        states_f[i] = std::vector<float_t>(states[i].begin(), states[i].end());
    }

    auto actions = this->monitor_.template get<action_type, 1>(batch);
    std::vector<std::vector<float_t>> actions_f(actions.size());

    for (uint_t i = 0; i < actions.size(); ++i)
    {
        actions_f[i] = std::vector<float_t>(actions[i].begin(), actions[i].end());
    }

    auto rewards_batch = this->monitor_.template get<float_t, 2>(batch);
    auto values_batch = this->monitor_.template get<torch_tensor_t, 5>(batch);
    auto logprobs_batch = this->monitor_.template get<torch_tensor_t, 4>(batch);

    // compute the discounted rewards for this batch
    auto discounted_returns =
        calculate_step_discounted_return(rewards_batch, static_cast<float_t>(this->config_.gamma));
    auto torch_states_batch = TorchAdaptor::stack(states_f, this->config_.device_type, true);
    auto torch_actions_batch = TorchAdaptor::stack(actions_f, this->config_.device_type, true);
    auto torch_rewards_batch =
        TorchAdaptor::to_torch(discounted_returns, this->config_.device_type, false).detach();
    auto torch_values_batch = TorchAdaptor::stack(values_batch, this->config_.device_type);
    auto old_torch_logprobs_batch =
        TorchAdaptor::stack(logprobs_batch, this->config_.device_type).detach();

    // form the advantage
    auto advantages = (torch_rewards_batch - torch_values_batch).detach();

    std::vector<real_t> loss_vals(this->config_.max_passes_over_batch, 0.0);
    for (uint_t p = 0; p < this->config_.max_passes_over_batch; ++p)
    {
        auto [new_log_probs, entropy, _] =
            this->policy_->evaluate(torch_states_batch, torch_actions_batch);

        auto ratio = (new_log_probs - old_torch_logprobs_batch).exp();
        auto surr1 = ratio * advantages;
        auto surr2 =
            torch::clamp(ratio, 1 - this->config_.clip_epsilon, 1 + this->config_.clip_epsilon) *
            advantages;
        auto actor_loss = -torch::min(surr1, surr2).mean();

        auto value_estimates = this->critic_->forward(torch_states_batch).squeeze();
        auto critic_loss = torch::nn::functional::mse_loss(value_estimates, torch_rewards_batch);

        auto total_loss = actor_loss + 0.5 * critic_loss - 0.01 * entropy;

        this->policy_optimizer_->zero_grad();
        this->critic_optimizer_->zero_grad();
        total_loss.backward();
        loss_vals[p] = total_loss.item().template to<real_t>();
        this->policy_optimizer_->step();
        this->critic_optimizer_->step();
    }

    // compute the undiscounted reward as the reward for this episode
    auto R = bitrl::maths::sum(rewards_batch);
    auto avg_loss = bitrl::maths::mean(loss_vals);
    this->monitor_.policy_loss_values.push_back(avg_loss);
    this->monitor_.rewards.push_back(R);
#ifdef BITRL_DEBUG
    BOOST_LOG_TRIVIAL(info) << "PPO: Done...: ";
#endif
    return std::make_tuple(R, avg_loss);
}

} // namespace rl::algos::pg
} // namespace bitrl
#endif
#endif
