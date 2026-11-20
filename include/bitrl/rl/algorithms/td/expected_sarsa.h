#ifndef EXPECTED_SARSA_H
#define EXPECTED_SARSA_H

#include "bitsim/bitsim_consts.h"
#include "cubeai/base/cubeai_config.h"
#include "cubeai/base/cubeai_types.h"
#include "cubeai/rl/algorithms/td/td_algo_base.h"
#include "cubeai/rl/worlds/envs_concepts.h"

#ifdef BITRL_DEBUG
#include <cassert>
#endif

namespace bitrl
{
namespace rl::algos::td
{

///
/// \brief The  ExpectedSARSA class. Simple implementation
/// of the expected SARSA algorithm
///
template <envs::discrete_world_concept EnvTp, typename ActionSelector>
class ExpectedSARSA : public TDAlgoBase<EnvTp>
{
  public:
    ///
    /// \brief env_t
    ///
    typedef typename TDAlgoBase<EnvTp>::env_type env_type;

    ///
    /// \brief action_t
    ///
    typedef typename TDAlgoBase<EnvTp>::action_type action_type;

    ///
    /// \brief state_t
    ///
    typedef typename TDAlgoBase<EnvTp>::state_type state_type;

    ///
    /// \brief action_selector_t
    ///
    typedef ActionSelector action_selector_type;

    ///
    /// \brief Constructor
    ///
    ExpectedSARSA(uint_t n_episodes, real_t tolerance, real_t gamma, real_t eta, uint_t plot_f,
                  env_type &env, uint_t max_num_iterations_per_episode,
                  const ActionSelector &selector);

    ///
    /// \brief Constructor
    ///
    ExpectedSARSA(TDAlgoConfig config, env_type &env, const ActionSelector &selector);

    ///
    /// \brief on_episode. Performs the iterations for
    /// one training episode
    ///
    virtual void on_episode() override final;

  private:
    ///
    /// \brief action_selector_
    ///
    action_selector_type action_selector_;

    ///
    /// \brief current_score_counter_
    ///
    uint_t current_score_counter_;

    ///
    /// \brief update_q_table_
    /// \param action
    ///
    void update_q_table_(const action_type &action, const state_type &cstate,
                         const state_type &next_state, const action_type &next_action,
                         real_t reward);
};

template <envs::discrete_world_concept EnvTp, typename ActionSelector>
ExpectedSARSA<EnvTp, ActionSelector>::ExpectedSARSA(uint_t n_episodes, real_t tolerance,
                                                    real_t gamma, real_t eta, uint_t plot_f,
                                                    env_type &env,
                                                    uint_t max_num_iterations_per_episode,
                                                    const ActionSelector &selector)
    : TDAlgoBase<EnvTp>(n_episodes, tolerance, gamma, eta, plot_f, max_num_iterations_per_episode,
                        env),
      action_selector_(selector), current_score_counter_(0)
{
}

template <envs::discrete_world_concept EnvTp, typename ActionSelector>
ExpectedSARSA<EnvTp, ActionSelector>::ExpectedSARSA(TDAlgoConfig config, env_type &env,
                                                    const ActionSelector &selector)
    : TDAlgoBase<EnvTp>(config, env), action_selector_(selector), current_score_counter_(0)
{
}

template <envs::discrete_world_concept EnvTp, typename ActionSelector>
void ExpectedSARSA<EnvTp, ActionSelector>::on_episode()
{

    // total score for the episode
    auto score = 0.0;
    auto state = this->env_ref_().reset().observation();

    // select an action
    auto action = action_selector_(this->q_table(), state);

    uint_t itr = 0;
    for (; itr < this->n_iterations_per_episode(); ++itr)
    {

        // select an action
        auto action = action_selector_(this->q_table(), state);
        if (this->is_verbose())
        {
            std::cout << "Episode iteration=" << itr << " of=" << this->n_iterations_per_episode()
                      << std::endl;
            std::cout << "State=" << state << std::endl;
            std::cout << "Action=" << action << std::endl;
        }

        // Take a on_episode
        auto step_type_result = this->env_ref_().step(action);

        auto next_state = step_type_result.observation();
        auto reward = step_type_result.reward();
        auto done = step_type_result.done();

        // accumulate score
        score += reward;

        if (!done)
        {
            auto next_action = action_selector_(this->q_table(), state);
            update_q_table_(action, state, next_state, next_action, reward);
            state = next_state;
            action = next_action;
        }
        else
        {

            update_q_table_(action, state, CubeAIConsts::invalid_size_type(),
                            CubeAIConsts::invalid_size_type(), reward);

            this->tmp_scores()[current_score_counter_++] = score;

            if (current_score_counter_ >= this->render_env_frequency_)
            {
                current_score_counter_ = 0;
            }

            if (this->is_verbose())
            {
                std::cout << "=============================================" << std::endl;
                std::cout << "Break out from episode=" << this->current_episode_idx() << std::endl;
                std::cout << "=============================================" << std::endl;
            }

            break;
        }
    }

    // make any adjustments to the way
    // actions are selected given the experience collected
    // in the episode
    action_selector_.adjust_on_episode(this->current_episode_idx());
    if (current_score_counter_ >= this->render_env_frequency_)
    {
        current_score_counter_ = 0;
    }

    std::cout << "Finished on_episode=" << this->current_episode_idx() << std::endl;
}

template <envs::discrete_world_concept EnvTp, typename ActionSelector>
void ExpectedSARSA<EnvTp, ActionSelector>::update_q_table_(const action_type &action,
                                                           const state_type &cstate,
                                                           const state_type &next_state,
                                                           const action_type &next_action,
                                                           real_t reward)
{

#ifdef BITRL_DEBUG
    assert(action < this->env_ref_().n_actions() && "Inavlid action idx");
    assert(cstate < this->env_ref_().n_states() && "Inavlid state idx");

    if (next_state != CubeAIConsts::invalid_size_type())
        assert(next_state < this->env_ref_().n_states() && "Inavlid next_state idx");

    if (next_action != CubeAIConsts::invalid_size_type())
        assert(next_action < this->env_ref_().n_actions() && "Inavlid next_action idx");
#endif

    const auto eps = action_selector_.eps_value();
    auto q_current = this->q_table()[cstate][action];
    auto policy_s = DynVec<real_t>(this->env_ref_().n_actions(), 1.0);
    policy_s *= eps / this->env_ref_().n_actions();

    auto state_action_values = this->q_table()[next_state];
    auto argmax = blaze::argmax(state_action_values);
    policy_s[argmax] = 1 - eps + (eps / this->env_ref_().n_actions());

    auto q_next = state_action_values * policy_s;
    auto td_target = reward + this->gamma() * q_next;
    // this->q_table()[cstate][action] = q_current + (this->eta() * (td_target - q_current));
}

} // namespace rl::algos::td
} // namespace bitrl

#endif // EXPECTED_SARSA_H
