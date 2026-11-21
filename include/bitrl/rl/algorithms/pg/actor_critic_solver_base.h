#ifndef ACTOR_CRITIC_SOLVER_BASE_H
#define ACTOR_CRITIC_SOLVER_BASE_H

///
/// Implements synchronous  advantage-actor critic, A2C, algorithm
/// Currently the implementation of this class assumes that
/// PyTorch is used to model the deep networks
///
///

#include "bitrl/base/bitrl_config.h"

#ifdef USE_PYTORCH

#include "bitrl/base/bitrl_log.h"
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

#include <any>
#include <chrono>
#include <exception>
#include <iostream>
#include <map>
#include <memory>
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
template <typename EnvType, typename PolicyType, typename CriticType, typename MonitorType,
          typename ConfigType>
class ACSolverBase : public RLSolverBase<EnvType>
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

    ///
    /// \brief The monitor type to use
    ///
    typedef MonitorType monitor_type;

    ///
    typedef typename monitor_type::experience_buffer_type experience_buffer_type;
    typedef typename monitor_type::experience_tuple_type experience_tuple_type;

    ///
    /// \brief The solver configuration type
    ///
    typedef ConfigType config_type;

    ///
    /// \brief Destructor
    ///
    virtual ~ACSolverBase() = default;

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
    virtual void actions_after_episode_ends(env_type &, uint_t /*episode_idx*/, const EpisodeInfo &)
    {
    }

    ///
    /// \brief actions_before_training_begins. Execute any actions the
    /// algorithm needs before starting the iterations
    ///
    virtual void actions_before_training_begins(env_type &);

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
    monitor_type &get_monitor() { return monitor_; }

  protected:
    ///
    /// \brief A2C
    /// \param config
    /// \param policy
    ///
    ACSolverBase(const config_type &config, policy_type &policy, critic_type &critic,
                 std::unique_ptr<torch::optim::Optimizer> &policy_optimizer,
                 std::unique_ptr<torch::optim::Optimizer> &critic_optimizer);

    ///
    /// \brief config_
    ///
    config_type config_;

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
    monitor_type monitor_;

    ///
    /// \brief The policy_ optimzer
    ///
    std::unique_ptr<torch::optim::Optimizer> policy_optimizer_;

    ///
    /// \brief The optimizer for the critic network
    ///
    std::unique_ptr<torch::optim::Optimizer> critic_optimizer_;

    ///
    /// @tparam ExperienceBufferType
    /// @param env
    /// @return
    uint_t create_episode_batch_(env_type &env, uint_t /*episode_idx*/,
                                 experience_buffer_type &buffer);
};

template <typename EnvType, typename PolicyType, typename CriticType, typename MonitorType,
          typename ConfigType>
ACSolverBase<EnvType, PolicyType, CriticType, MonitorType, ConfigType>::ACSolverBase(
    const config_type &config, policy_type &policy, critic_type &critic,
    std::unique_ptr<torch::optim::Optimizer> &policy_optimizer,
    std::unique_ptr<torch::optim::Optimizer> &critic_optimizer)
    : RLSolverBase<EnvType>(), config_(config), policy_(policy), critic_(critic), monitor_(),
      policy_optimizer_(std::move(policy_optimizer)), critic_optimizer_(std::move(critic_optimizer))
{
}

template <typename EnvType, typename PolicyType, typename CriticType, typename MonitorType,
          typename ConfigType>
void ACSolverBase<EnvType, PolicyType, CriticType, MonitorType,
                  ConfigType>::set_train_mode() noexcept
{
    policy_->train();
    critic_->train();
}

template <typename EnvType, typename PolicyType, typename CriticType, typename MonitorType,
          typename ConfigType>
void ACSolverBase<EnvType, PolicyType, CriticType, MonitorType,
                  ConfigType>::set_evaluation_mode() noexcept
{
    policy_->eval();
    critic_->eval();
}

template <typename EnvType, typename PolicyType, typename CriticType, typename MonitorType,
          typename ConfigType>
void ACSolverBase<EnvType, PolicyType, CriticType, MonitorType,
                  ConfigType>::actions_before_training_begins(env_type & /*env*/)
{

    monitor_.reset();
    monitor_.policy_loss_values.reserve(config_.n_episodes);
    monitor_.critic_loss_values.reserve(config_.n_episodes);
    monitor_.rewards.reserve(config_.n_episodes);
    monitor_.episode_duration.reserve(config_.n_episodes);
    set_train_mode();
}

template <typename EnvType, typename PolicyType, typename CriticType, typename MonitorType,
          typename ConfigType>
uint_t
ACSolverBase<EnvType, PolicyType, CriticType, MonitorType, ConfigType>::create_episode_batch_(
    env_type &env, uint_t episode_idx, experience_buffer_type &buffer)
{

#ifdef BITRL_DEBUG
    BITRL_LOG_INFO << "Collecting batch for episode: " << episode_idx;
#endif

    /// we want to push into the monitor
    /// experience tuples

    typedef typename MonitorType::experience_tuple_type experience_tuple_type;

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

        // if the step is done then break
        if (next_time_step.done())
        {
            break;
        }

        old_timestep = next_time_step;
    }

#ifdef BITRL_DEBUG
    BITRL_LOG_INFO << "Done... ";
#endif

    return itrs + 1;
}

} // namespace pg
} // namespace algos
} // namespace rl
} // namespace bitrl

#endif // USE_PYTORCH
#endif
