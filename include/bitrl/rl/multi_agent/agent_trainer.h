#ifndef AGENT_TRAINER_H
#define AGENT_TRAINER_H

#include "bitrl/base/bitrl_types.h"
#include "bitrl/rl/multi_agent/multi_agent_trainer.h"

#include <chrono>
#include <concepts>
#include <functional>

namespace bitrl
{
namespace rl::multi_agent
{

///
/// \brief An environment seen by one of its agents: it takes this agent's action alone, and
/// gives the multi-agent time steps; bitrl::envs::multi_agent::DistributedAgentEnv is one.
///
template <typename EnvType>
concept agent_env = requires(EnvType env, const typename EnvType::action_type &action)
{
    {
        env.reset()
        } -> std::convertible_to<typename EnvType::time_step_type>;
    {
        env.step(action)
        } -> std::convertible_to<typename EnvType::time_step_type>;
};

///
/// \brief Something that chooses the action of one agent from a multi-agent time step
///
template <typename ActorType, typename EnvType>
concept agent_actor = requires(ActorType actor, const typename EnvType::time_step_type &step)
{
    {
        actor.act(step)
        } -> std::convertible_to<typename EnvType::action_type>;
};

///
/// \brief One agent that learns from its transitions, and hears when an episode begins and ends;
/// QLearningAgent is one
///
template <typename LearnerType, typename EnvType>
concept agent_learner = agent_actor<LearnerType, EnvType> &&
    requires(LearnerType learner, const typename EnvType::time_step_type &step,
             const typename EnvType::action_type &action, uint_t episode)
{
    learner.update(step, action, step);
    learner.on_episode_begin(episode);
    learner.on_episode_end(episode);
};

///
/// \brief Play one episode of env with the action of one agent chosen by actor.
///
/// The result has the returns of every agent, which the time steps give.
///
template <typename EnvType, typename ActorType>
requires agent_env<EnvType> && agent_actor<ActorType, EnvType> MultiAgentEpisodeResult
run_agent_episode(EnvType &env, ActorType &actor,
                  const std::function<void(const typename EnvType::time_step_type &)> &on_step = {})
{
    MultiAgentEpisodeResult result;
    auto step = env.reset();
    result.returns.assign(step.n_agents(), 0.0);
    if (on_step)
    {
        on_step(step);
    }
    while (!step.last())
    {
        step = env.step(actor.act(step));
        detail::accumulate_step(result, step);
        if (on_step)
        {
            on_step(step);
        }
    }
    result.all_done = detail::all_done(step);
    return result;
}

///
/// \brief Train one agent of a multi-agent environment, an episode at a time.
///
/// The same loop as MultiAgentTrainer, for a learner of one agent whose environment, such as a
/// bitrl::envs::multi_agent::DistributedAgentEnv, gets the actions of the other agents
/// elsewhere. Every participant of a distributed training runs one, with the same number of
/// episodes.
///
template <typename EnvType, typename LearnerType>
requires agent_env<EnvType> && agent_learner<LearnerType, EnvType>
class AgentTrainer
{
  public:
    AgentTrainer(const MultiAgentTrainerConfig &config, LearnerType &learner)
        : config_(config), learner_(learner)
    {
    }

    ///
    /// \brief Run config.n_episodes episodes of training
    ///
    MultiAgentTrainingResult train(EnvType &env)
    {
        const auto start = std::chrono::steady_clock::now();
        MultiAgentTrainingResult training;
        training.episodes.reserve(config_.n_episodes);
        for (uint_t episode = 0; episode < config_.n_episodes; ++episode)
        {
            learner_.on_episode_begin(episode);
            MultiAgentEpisodeResult result;
            auto step = env.reset();
            result.returns.assign(step.n_agents(), 0.0);
            while (!step.last())
            {
                const auto action = learner_.act(step);
                auto next = env.step(action);
                learner_.update(step, action, next);
                detail::accumulate_step(result, next);
                step = std::move(next);
            }
            result.all_done = detail::all_done(step);
            learner_.on_episode_end(episode);
            training.episodes.push_back(std::move(result));
        }
        training.total_time = std::chrono::steady_clock::now() - start;
        return training;
    }

  private:
    MultiAgentTrainerConfig config_;
    LearnerType &learner_;
};

} // namespace rl::multi_agent
} // namespace bitrl

#endif // AGENT_TRAINER_H
