#ifndef MULTI_AGENT_TRAINER_H
#define MULTI_AGENT_TRAINER_H

#include "bitrl/base/bitrl_log.h"
#include "bitrl/base/bitrl_types.h"
#include "bitsim/bitsim_consts.h"

#include <chrono>
#include <concepts>
#include <functional>
#include <vector>

namespace bitrl
{
namespace rl::multi_agent
{

///
/// \brief Something that chooses one action per agent from a multi-agent time step: a team of
/// controllers, or of learners.
///
template <typename ActorType, typename EnvType>
concept multi_agent_actor = requires(ActorType actor, const typename EnvType::time_step_type &step)
{
    {
        actor.act(step)
        } -> std::convertible_to<typename EnvType::joint_action_type>;
};

///
/// \brief A team that learns: it acts, learns from every joint transition, and hears when an
/// episode begins and ends.
///
template <typename LearnerType, typename EnvType>
concept multi_agent_learner = multi_agent_actor<LearnerType, EnvType> &&
    requires(LearnerType learner, const typename EnvType::time_step_type &step,
             const typename EnvType::joint_action_type &actions, uint_t episode)
{
    learner.update(step, actions, step);
    learner.on_episode_begin(episode);
    learner.on_episode_end(episode);
};

///
/// \brief What one episode of a multi-agent environment gave
///
struct MultiAgentEpisodeResult
{
    std::vector<real_t> returns; ///< the undiscounted return of every agent
    uint_t n_steps{0};           ///< the joint steps of the episode
    uint_t n_collisions{0};      ///< the sum of the "collisions" the time steps reported
    bool all_done{false};        ///< whether every agent finished, rather than the step limit

    ///
    /// \brief The sum of the returns of the agents
    ///
    real_t team_return() const noexcept;
};

///
/// \brief Play one episode: reset env, then let actor choose the joint actions until the episode
/// is over.
///
/// \param on_step called with every time step, the first one included, to record the episode
///
template <typename EnvType, typename ActorType>
requires multi_agent_actor<ActorType, EnvType> MultiAgentEpisodeResult
run_episode(EnvType &env, ActorType &actor,
            const std::function<void(const typename EnvType::time_step_type &)> &on_step = {});

///
/// \brief The settings of MultiAgentTrainer
///
struct MultiAgentTrainerConfig
{
    uint_t n_episodes{1000};
    uint_t output_msg_frequency{bitsim::consts::INVALID_ID}; ///< log every so many episodes
};

///
/// \brief What MultiAgentTrainer::train() gave: an entry per episode
///
struct MultiAgentTrainingResult
{
    std::vector<MultiAgentEpisodeResult> episodes;
    std::chrono::duration<real_t> total_time{0.0};
};

///
/// \brief Train a team of learners on a multi-agent environment, an episode at a time.
///
/// Every step the team chooses the joint action, the environment applies it, and the team learns
/// from the transition. The team decides how its agents learn: independently, as
/// IndependentQLearning does, or from the joint experience.
///
template <typename EnvType, typename LearnerType>
requires multi_agent_learner<LearnerType, EnvType>
class MultiAgentTrainer
{
  public:
    typedef EnvType env_type;
    typedef LearnerType learner_type;

    MultiAgentTrainer(const MultiAgentTrainerConfig &config, learner_type &learner)
        : config_(config), learner_(learner)
    {
    }

    ///
    /// \brief Run config.n_episodes episodes of training
    ///
    MultiAgentTrainingResult train(env_type &env);

  private:
    MultiAgentTrainerConfig config_;
    learner_type &learner_;
};

inline real_t MultiAgentEpisodeResult::team_return() const noexcept
{
    real_t total = 0.0;
    for (const real_t r : returns)
    {
        total += r;
    }
    return total;
}

namespace detail
{
template <typename TimeStepType>
void accumulate_step(MultiAgentEpisodeResult &result, const TimeStepType &step)
{
    for (uint_t a = 0; a < step.n_agents(); ++a)
    {
        result.returns[a] += step.reward(a);
    }
    if (step.extra().contains("collisions"))
    {
        result.n_collisions += step.template get_extra<uint_t>("collisions");
    }
    ++result.n_steps;
}

template <typename TimeStepType> bool all_done(const TimeStepType &step)
{
    for (uint_t a = 0; a < step.n_agents(); ++a)
    {
        if (!step.done(a))
        {
            return false;
        }
    }
    return true;
}
} // namespace detail

template <typename EnvType, typename ActorType>
requires multi_agent_actor<ActorType, EnvType> MultiAgentEpisodeResult
run_episode(EnvType &env, ActorType &actor,
            const std::function<void(const typename EnvType::time_step_type &)> &on_step)
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

template <typename EnvType, typename LearnerType>
requires multi_agent_learner<LearnerType, EnvType>
    MultiAgentTrainingResult MultiAgentTrainer<EnvType, LearnerType>::train(env_type &env)
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
            const auto actions = learner_.act(step);
            auto next = env.step(actions);
            learner_.update(step, actions, next);
            detail::accumulate_step(result, next);
            step = std::move(next);
        }
        result.all_done = detail::all_done(step);

        learner_.on_episode_end(episode);

        if (config_.output_msg_frequency != bitsim::consts::INVALID_ID &&
            config_.output_msg_frequency > 0 && episode % config_.output_msg_frequency == 0)
        {
            BITRL_LOG_INFO << "Episode " << episode << ": team return " << result.team_return()
                           << ", steps " << result.n_steps << ", collisions "
                           << result.n_collisions;
        }
        training.episodes.push_back(std::move(result));
    }

    training.total_time = std::chrono::steady_clock::now() - start;
    return training;
}

} // namespace rl::multi_agent
} // namespace bitrl

#endif // MULTI_AGENT_TRAINER_H
