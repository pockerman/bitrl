#ifndef POLICY_IMPROVEMENT_H
#define POLICY_IMPROVEMENT_H

#include "bitrl/rl/algorithms/dp/dp_algo_base.h"
#include "bitrl/rl/algorithms/utils.h"
#include "bitrl/rl/policies/adaptors/policy_stochastic_adaptor.h"

#include <any>
#include <map>
#include <string>

namespace bitrl
{
namespace rl::algos::dp
{

/**
 * @brief The PolicyImprovement class. PolicyImprovement is not a real
 *  algorithm in the sense that it looks for a policy. Instead, it is
 * more of a helper function that allows as to improve on a given policy.
 */
template <typename EnvType, typename PolicyType>
class PolicyImprovement : public DPSolverBase<EnvType>
{
  public:
    ///
    /// \brief env_t
    ///
    typedef typename DPSolverBase<EnvType>::env_type env_type;

    ///
    /// \brief policy_type
    ///
    typedef PolicyType policy_type;

    ///
    /// \brief IterativePolicyEval
    ///
    PolicyImprovement(uint_t action_space_size, real_t gamma, const DynVec<real_t> &val_func,
                      policy_type &policy);

    ///
    /// \brief actions_before_training_begins. Execute any actions the
    /// algorithm needs before starting the iterations
    ///
    virtual void actions_before_training_begins(env_type & /*env*/) override {}

    ///
    /// \brief actions_after_training_ends. Actions to execute after
    /// the training iterations have finisehd
    ///
    virtual void actions_after_training_ends(env_type & /*env*/) override {}

    ///
    /// \brief actions_before_training_episode
    ///
    virtual void actions_before_episode_begins(env_type &, uint_t /*episode_idx*/) override {}

    ///
    /// \brief actions_after_training_episode
    ///
    virtual void actions_after_episode_ends(env_type &, uint_t /*episode_idx*/,
                                            const EpisodeInfo & /*einfo*/) override
    {
    }

    ///
    /// \brief on_episode Do one on_episode of the algorithm
    ///
    virtual EpisodeInfo on_training_episode(env_type &env, uint_t episode_idx) override;

    ///
    /// \brief policy
    /// \return
    ///
    const policy_type &policy() const { return policy_; }

    ///
    /// \brief policy
    /// \return
    ///
    policy_type &policy() { return policy_; }

    ///
    /// \brief set_value_function
    /// \param v
    ///
    void set_value_function(const DynVec<real_t> &v) { v_ = v; }

  protected:
    ///
    /// \brief gamma_
    ///
    real_t gamma_;

    ///
    /// \brief v_
    ///
    DynVec<real_t> v_;

    ///
    /// \brief policy_
    ///
    policy_type &policy_;

    ///
    /// \brief How to adapt the policy
    ///
    bitrl::rl::policies::StochasticAdaptorPolicy<policy_type> policy_adaptor_;
};

template <typename EnvType, typename PolicyType>
PolicyImprovement<EnvType, PolicyType>::PolicyImprovement(uint_t action_space_size, real_t gamma,
                                                          const DynVec<real_t> &val_func,
                                                          policy_type &policy)
    : DPSolverBase<EnvType>(), gamma_(gamma), v_(val_func), policy_(policy),
      policy_adaptor_(val_func.size(), action_space_size, policy)
{
}

template <typename EnvType, typename PolicyType>
EpisodeInfo PolicyImprovement<EnvType, PolicyType>::on_training_episode(env_type &env,
                                                                        uint_t episode_idx)
{

    auto start = std::chrono::steady_clock::now();

    std::map<std::string, std::any> options;

    for (uint_t s = 0; s < env.n_states(); ++s)
    {

        auto state_actions = state_actions_from_v(env, v_, gamma_, s);

        options.insert_or_assign("state", s);
        options.insert_or_assign("state_actions", std::any(state_actions));
        policy_ = policy_adaptor_(options);
    }

    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<real_t> elapsed_seconds = end - start;

    EpisodeInfo info;
    info.episode_index = episode_idx;
    info.episode_iterations = env.n_states();
    info.total_time = elapsed_seconds;
    return info;
}

} // namespace rl::algos::dp
} // namespace bitrl

#endif // POLICY_IMPROVEMENT_H
