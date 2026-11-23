#ifndef PARALLEL_PPO_H
#define PARALLEL_PPO_H

#include "bitrl/base/bitrl_config.h"

#ifdef USE_PYTORCH

#include "bitrl/base/bitrl_types.h"
#include "bitrl/envs/parallel_envs.h"
#include "bitrl/rl/algorithms/pg/a2c_monitor.h"
#include "bitrl/rl/algorithms/pg/actor_critic_solver_base.h"
#include "bitrl/rl/algorithms/pg/ppo_config.h"
#include "bitrl/rl/episode_info.h"

#include <torch/torch.h>

#include <chrono>
#include <memory>
#include <tuple>
#include <type_traits>
#include <vector>

namespace bitrl
{
namespace rl::algos::pg
{

///
/// \brief Proximal policy optimisation that learns from several episodes at once, played by
/// the copies of a bitrl::envs::ParallelEnvs.
///
/// Every training round, on_training_episode(), resets every copy and plays an episode in
/// each, all at the same time: at every step the policy acts for all the copies whose episode is
/// not over with one call on a batch of their states, and ParallelEnvs steps the copies on its
/// threads. A copy's episode ends when it is done or after config.max_itrs_per_episode steps.
/// Then the networks learn from all the episodes together, with the clipped objective of
/// PPOSolver: config.max_passes_over_batch passes over the whole batch, the advantage being the
/// discounted return of the step, within its own episode, minus the critic's value, normalised
/// if config.normalize_advantages, and the loss actor + 0.5 critic - 0.01 entropy.
///
/// A round of N copies plays N episodes; the actions are sampled on this thread with libtorch's
/// generator, so a training is the same whatever the number of threads, and repeatable with
/// torch::manual_seed().
///
/// The policy must have
///
///   std::tuple<torch::Tensor, torch::Tensor> act_batch(torch::Tensor states)
///
/// that gives an action and its log probability for every row of states, and the evaluate()
/// of PPOSolver; the critic must have forward(states), a value per row.
///
/// \tparam EnvType the environment of a copy; the solver trains on ParallelEnvs<EnvType>
///
template <typename EnvType, typename PolicyType, typename CriticType>
class ParallelPPOSolver final
    : public ACSolverBase<envs::ParallelEnvs<EnvType>, PolicyType, CriticType,
                          A2CMonitor<typename EnvType::action_type, typename EnvType::state_type>,
                          PPOConfig>
{
  public:
    typedef envs::ParallelEnvs<EnvType> env_type;
    typedef PolicyType policy_type;
    typedef CriticType critic_type;
    typedef typename EnvType::state_type state_type;
    typedef typename EnvType::action_type action_type;
    typedef ACSolverBase<env_type, PolicyType, CriticType, A2CMonitor<action_type, state_type>,
                         PPOConfig>
        base_type;

    ParallelPPOSolver(const PPOConfig &config, policy_type &policy, critic_type &critic,
                      std::unique_ptr<torch::optim::Optimizer> &policy_optimizer,
                      std::unique_ptr<torch::optim::Optimizer> &critic_optimizer)
        : base_type(config, policy, critic, policy_optimizer, critic_optimizer)
    {
    }

    ///
    /// \brief Play an episode in every copy and learn from all of them; the reward of the
    /// EpisodeInfo is the mean return of the episodes, its iterations the steps of all of them
    ///
    EpisodeInfo on_training_episode(env_type &envs, uint_t round) override;

    ///
    /// \brief The return of every episode of the last round
    ///
    const std::vector<real_t> &last_returns() const noexcept { return last_returns_; }

  private:
    // what one copy played
    struct Episode
    {
        std::vector<std::vector<float_t>> states;
        std::vector<std::vector<float_t>> actions;
        std::vector<real_t> rewards;
        std::vector<float_t> log_probs;
        std::vector<float_t> values;
    };

    template <typename T> static std::vector<float_t> as_floats_(const T &value);

    static action_type action_from_(const torch::Tensor &actions, int64_t row);

    real_t learn_(const std::vector<Episode> &episodes);

    std::vector<real_t> last_returns_;
};

template <typename EnvType, typename PolicyType, typename CriticType>
template <typename T>
std::vector<float_t> ParallelPPOSolver<EnvType, PolicyType, CriticType>::as_floats_(const T &value)
{
    if constexpr (std::is_arithmetic_v<T>)
    {
        return {static_cast<float_t>(value)};
    }
    else
    {
        return std::vector<float_t>(value.begin(), value.end());
    }
}

template <typename EnvType, typename PolicyType, typename CriticType>
typename ParallelPPOSolver<EnvType, PolicyType, CriticType>::action_type
ParallelPPOSolver<EnvType, PolicyType, CriticType>::action_from_(const torch::Tensor &actions,
                                                                 int64_t row)
{
    const auto a = actions[row].to(torch::kDouble).contiguous();
    if constexpr (std::is_arithmetic_v<action_type>)
    {
        return static_cast<action_type>(a.template item<double>());
    }
    else
    {
        const auto flat = a.view({-1});
        action_type out(static_cast<std::size_t>(flat.numel()));
        for (int64_t k = 0; k < flat.numel(); ++k)
        {
            out[k] = static_cast<typename action_type::value_type>(flat[k].template item<double>());
        }
        return out;
    }
}

template <typename EnvType, typename PolicyType, typename CriticType>
EpisodeInfo ParallelPPOSolver<EnvType, PolicyType, CriticType>::on_training_episode(env_type &envs,
                                                                                    uint_t round)
{
    const auto start = std::chrono::steady_clock::now();
    const uint_t n = envs.size();
    std::vector<Episode> episodes(n);
    uint_t n_steps = 0;

    {
        torch::NoGradGuard no_grad;
        envs.reset();
        std::vector<action_type> actions(n, action_type{});
        for (uint_t t = 0; t < this->config_.max_itrs_per_episode && !envs.all_done(); ++t)
        {
            // the copies still playing, and their states, in one batch
            std::vector<uint_t> active;
            std::vector<std::vector<float_t>> rows;
            for (uint_t i = 0; i < n; ++i)
            {
                if (!envs.done(i))
                {
                    active.push_back(i);
                    rows.push_back(as_floats_(envs.time_steps()[i].observation()));
                }
            }
            const auto width = static_cast<int64_t>(rows.front().size());
            std::vector<float_t> flat;
            flat.reserve(rows.size() * rows.front().size());
            for (const auto &row : rows)
            {
                flat.insert(flat.end(), row.begin(), row.end());
            }
            const auto states =
                torch::from_blob(flat.data(), {static_cast<int64_t>(rows.size()), width},
                                 torch::TensorOptions().dtype(torch::kFloat32))
                    .clone();

            auto [batch_actions, log_probs] = this->policy_->act_batch(states);
            const auto values = this->critic_->forward(states).view({-1});
            const auto lp = log_probs.view({-1}).to(torch::kFloat32).contiguous();
            const auto v = values.to(torch::kFloat32).contiguous();

            for (std::size_t r = 0; r < active.size(); ++r)
            {
                actions[active[r]] = action_from_(batch_actions, static_cast<int64_t>(r));
            }
            const auto &next = envs.step(actions);
            for (std::size_t r = 0; r < active.size(); ++r)
            {
                auto &e = episodes[active[r]];
                e.states.push_back(rows[r]);
                e.actions.push_back(as_floats_(actions[active[r]]));
                e.rewards.push_back(next[active[r]].reward());
                e.log_probs.push_back(lp[static_cast<int64_t>(r)].template item<float_t>());
                e.values.push_back(v[static_cast<int64_t>(r)].template item<float_t>());
            }
            n_steps += static_cast<uint_t>(active.size());
        }
    }

    last_returns_.assign(n, 0.0);
    real_t mean_return = 0.0;
    for (uint_t i = 0; i < n; ++i)
    {
        for (const real_t r : episodes[i].rewards)
        {
            last_returns_[i] += r;
        }
        mean_return += last_returns_[i] / n;
    }

    const real_t loss = learn_(episodes);
    this->monitor_.rewards.push_back(mean_return);
    this->monitor_.policy_loss_values.push_back(loss);
    this->monitor_.episode_duration.push_back(n_steps);

    EpisodeInfo info;
    info.episode_index = round;
    info.episode_reward = mean_return;
    info.episode_iterations = n_steps;
    info.total_time = std::chrono::steady_clock::now() - start;
    return info;
}

template <typename EnvType, typename PolicyType, typename CriticType>
real_t
ParallelPPOSolver<EnvType, PolicyType, CriticType>::learn_(const std::vector<Episode> &episodes)
{
    // the discounted return of every step, within its own episode
    std::vector<float_t> returns, old_values, old_log_probs, state_data, action_data;
    int64_t n_rows = 0, state_width = 0, action_width = 0;
    for (const auto &e : episodes)
    {
        std::vector<float_t> g(e.rewards.size());
        real_t running = 0.0;
        for (std::size_t k = e.rewards.size(); k-- > 0;)
        {
            running = e.rewards[k] + this->config_.gamma * running;
            g[k] = static_cast<float_t>(running);
        }
        returns.insert(returns.end(), g.begin(), g.end());
        old_values.insert(old_values.end(), e.values.begin(), e.values.end());
        old_log_probs.insert(old_log_probs.end(), e.log_probs.begin(), e.log_probs.end());
        for (std::size_t k = 0; k < e.states.size(); ++k)
        {
            state_width = static_cast<int64_t>(e.states[k].size());
            action_width = static_cast<int64_t>(e.actions[k].size());
            state_data.insert(state_data.end(), e.states[k].begin(), e.states[k].end());
            action_data.insert(action_data.end(), e.actions[k].begin(), e.actions[k].end());
            ++n_rows;
        }
    }
    if (n_rows == 0)
    {
        return 0.0;
    }

    const auto f32 = torch::TensorOptions().dtype(torch::kFloat32);
    const auto states = torch::from_blob(state_data.data(), {n_rows, state_width}, f32).clone();
    const auto actions = torch::from_blob(action_data.data(), {n_rows, action_width}, f32).clone();
    const auto g = torch::from_blob(returns.data(), {n_rows}, f32).clone();
    const auto values = torch::from_blob(old_values.data(), {n_rows}, f32).clone();
    const auto old_lp = torch::from_blob(old_log_probs.data(), {n_rows}, f32).clone();

    auto advantages = g - values;
    if (this->config_.normalize_advantages && n_rows > 1)
    {
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8);
    }

    real_t total = 0.0;
    for (uint_t p = 0; p < this->config_.max_passes_over_batch; ++p)
    {
        auto [new_log_probs, entropy, _] = this->policy_->evaluate(states, actions);
        const auto ratio = (new_log_probs.view({-1}) - old_lp).exp();
        const auto surr1 = ratio * advantages;
        const auto surr2 =
            torch::clamp(ratio, 1 - this->config_.clip_epsilon, 1 + this->config_.clip_epsilon) *
            advantages;
        const auto actor_loss = -torch::min(surr1, surr2).mean();
        const auto critic_loss =
            torch::nn::functional::mse_loss(this->critic_->forward(states).view({-1}), g);
        const auto loss = actor_loss + 0.5 * critic_loss - 0.01 * entropy;

        this->policy_optimizer_->zero_grad();
        this->critic_optimizer_->zero_grad();
        loss.backward();
        total += loss.template item<real_t>();
        this->policy_optimizer_->step();
        this->critic_optimizer_->step();
    }
    return this->config_.max_passes_over_batch ? total / this->config_.max_passes_over_batch : 0.0;
}

} // namespace rl::algos::pg
} // namespace bitrl

#endif // USE_PYTORCH
#endif // PARALLEL_PPO_H
