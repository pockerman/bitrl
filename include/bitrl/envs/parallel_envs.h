#ifndef BITRL_PARALLEL_ENVS_H
#define BITRL_PARALLEL_ENVS_H

#include "bitrl/base/bitrl_types.h"
#include "bitsim/utils/concurrency/parallel.h"
#include "bitsim/utils/concurrency/thread_pool.h"

#include <any>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace bitrl
{
namespace envs
{

///
/// \brief Several copies of an environment, stepped at the same time on the threads of a
/// bitsim::utils::concurrency::ThreadPool.
///
/// step() takes an action per copy and steps every copy whose episode is not over; a copy whose
/// episode is over keeps its last time step, and ignores its action, until the next reset().
/// Every copy is an environment of its own, so the copies share nothing, and what they do does
/// not depend on the threads: a run with one thread is the same as one with many.
///
/// Stepping on threads pays off for environments that take time: a physics engine, a server
/// over the network. For a cheap environment, step them on this thread, with use_threads(false);
/// what is left to gain then is in acting for all the copies at once, as ParallelPPOSolver does.
///
/// \tparam EnvType an environment in the form of bitrl::envs::EnvBase
///
template <typename EnvType> class ParallelEnvs
{
  public:
    typedef EnvType env_type;
    typedef typename EnvType::time_step_type time_step_type;
    typedef typename EnvType::state_type state_type;
    typedef typename EnvType::action_type action_type;

    ///
    /// \brief n copies, each built with args
    ///
    template <typename... Args> explicit ParallelEnvs(uint_t n, Args &&...args);

    ///
    /// \brief Make every copy with the same options; copy i resets first with the seed
    /// task_seed(seed, i), where seed is the reset option "seed" (42 if not given)
    ///
    void make(const std::string &version,
              const std::unordered_map<std::string, std::any> &make_options,
              const std::unordered_map<std::string, std::any> &reset_options);

    ///
    /// \brief Start a new episode in every copy, each with its next seed
    ///
    const std::vector<time_step_type> &reset();

    ///
    /// \brief Step every copy whose episode is not over with its action
    ///
    const std::vector<time_step_type> &step(const std::vector<action_type> &actions);

    uint_t size() const noexcept { return static_cast<uint_t>(envs_.size()); }

    ///
    /// \brief Whether the episode of copy i is over
    ///
    bool done(uint_t i) const { return time_steps_.at(i).last(); }

    ///
    /// \brief Whether the episode of every copy is over
    ///
    bool all_done() const;

    const std::vector<time_step_type> &time_steps() const noexcept { return time_steps_; }

    EnvType &env(uint_t i) { return *envs_.at(i); }

    const EnvType &env(uint_t i) const { return *envs_.at(i); }

    ///
    /// \brief Step on the threads of pool, the default pool if not set
    ///
    void set_pool(bitsim::utils::concurrency::ThreadPool &pool) noexcept { pool_ = &pool; }

    ///
    /// \brief Step on threads, or on this thread one copy after the other
    ///
    void use_threads(bool threads) noexcept { use_threads_ = threads; }

    bool is_created() const noexcept { return is_created_; }

    ///
    /// \brief Close every copy
    ///
    void close();

  private:
    template <typename F> void for_each_(F &&f);

    std::vector<std::unique_ptr<EnvType>> envs_;
    std::vector<time_step_type> time_steps_;
    bitsim::utils::concurrency::ThreadPool *pool_{nullptr};
    bool use_threads_{true};
    bool is_created_{false};
};

template <typename EnvType>
template <typename... Args>
ParallelEnvs<EnvType>::ParallelEnvs(uint_t n, Args &&...args)
{
    if (n == 0)
    {
        throw std::invalid_argument("ParallelEnvs: needs at least one copy");
    }
    envs_.reserve(n);
    for (uint_t i = 0; i < n; ++i)
    {
        envs_.push_back(std::make_unique<EnvType>(args...));
    }
    time_steps_.resize(n);
}

template <typename EnvType> template <typename F> void ParallelEnvs<EnvType>::for_each_(F &&f)
{
    if (!use_threads_ || envs_.size() == 1)
    {
        for (uint_t i = 0; i < size(); ++i)
        {
            f(i);
        }
        return;
    }
    auto &pool = pool_ ? *pool_ : bitsim::utils::concurrency::default_thread_pool();
    bitsim::utils::concurrency::parallel_for(uint_t(0), size(), f, pool);
}

template <typename EnvType>
void ParallelEnvs<EnvType>::make(const std::string &version,
                                 const std::unordered_map<std::string, std::any> &make_options,
                                 const std::unordered_map<std::string, std::any> &reset_options)
{
    uint_t seed = 42;
    if (const auto itr = reset_options.find("seed"); itr != reset_options.end())
    {
        seed = std::any_cast<uint_t>(itr->second);
    }
    // one after the other: making an environment may not be safe on threads
    for (uint_t i = 0; i < size(); ++i)
    {
        auto options = reset_options;
        options["seed"] = std::any(bitsim::utils::concurrency::task_seed(seed, i));
        envs_[i]->make(version, make_options, options);
    }
    is_created_ = true;
}

template <typename EnvType>
const std::vector<typename ParallelEnvs<EnvType>::time_step_type> &ParallelEnvs<EnvType>::reset()
{
    if (!is_created_)
    {
        throw std::logic_error("ParallelEnvs: call make() before reset()");
    }
    for_each_([this](uint_t i) { time_steps_[i] = envs_[i]->reset(); });
    return time_steps_;
}

template <typename EnvType>
const std::vector<typename ParallelEnvs<EnvType>::time_step_type> &
ParallelEnvs<EnvType>::step(const std::vector<action_type> &actions)
{
    if (actions.size() != envs_.size())
    {
        throw std::invalid_argument("ParallelEnvs: step() needs an action per copy, " +
                                    std::to_string(envs_.size()) + ", but got " +
                                    std::to_string(actions.size()));
    }
    for_each_(
        [&](uint_t i)
        {
            if (!time_steps_[i].last())
            {
                time_steps_[i] = envs_[i]->step(actions[i]);
            }
        });
    return time_steps_;
}

template <typename EnvType> bool ParallelEnvs<EnvType>::all_done() const
{
    for (const auto &t : time_steps_)
    {
        if (!t.last())
        {
            return false;
        }
    }
    return true;
}

template <typename EnvType> void ParallelEnvs<EnvType>::close()
{
    for (auto &env : envs_)
    {
        env->close();
    }
    is_created_ = false;
}

} // namespace envs
} // namespace bitrl

#endif // BITRL_PARALLEL_ENVS_H
