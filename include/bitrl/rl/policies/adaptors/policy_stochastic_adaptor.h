#ifndef STOCHASTIC_ADAPTOR_POLICY_H
#define STOCHASTIC_ADAPTOR_POLICY_H

#include "bitrl/base/bitrl_config.h"
#include "bitrl/base/bitrl_types.h"
#include "bitrl/maths/vector_math.h"

#ifdef BITRL_DEBUG
#include <cassert>
#endif

#include <any>
#include <map>
#include <memory>

namespace bitrl
{
namespace rl
{
namespace policies
{

class DiscretePolicyBase;

///
/// \brief The StochasticAdaptorPolicy class
///
template <typename PolicyType> class StochasticAdaptorPolicy /*: public DiscretePolicyAdaptorBase*/
{

  public:
    typedef PolicyType policy_type;

    ///
    /// \brief StochasticAdaptorPolicy
    /// \param policy
    ///
    StochasticAdaptorPolicy(uint_t state_space_size, uint_t action_space_size, policy_type &policy);

    ///
    /// \brief Destructor
    ///
    ~StochasticAdaptorPolicy() = default;

    ///
    /// \brief operator ()
    /// \param options
    /// \return
    ///
    virtual policy_type &operator()(const std::map<std::string, std::any> &options);

  private:
    ///
    /// \brief state_space_size_
    ///
    uint_t state_space_size_;

    ///
    /// \brief action_space_size_
    ///
    uint_t action_space_size_;

    ///
    /// \brief policy_
    ///
    policy_type &policy_;
};

template <typename PolicyType>
StochasticAdaptorPolicy<PolicyType>::StochasticAdaptorPolicy(uint_t state_space_size,
                                                             uint_t action_space_size,
                                                             policy_type &policy)
    : // DiscretePolicyAdaptorBase(),
      state_space_size_(state_space_size), action_space_size_(action_space_size), policy_(policy)
{
}

template <typename PolicyType>
typename StochasticAdaptorPolicy<PolicyType>::policy_type &
StochasticAdaptorPolicy<PolicyType>::operator()(const std::map<std::string, std::any> &options)
{

    auto state = std::any_cast<uint_t>(options.find("state")->second);
    auto state_actions = std::any_cast<DynVec<real_t>>(options.find("state_actions")->second);
    auto best_actions = maths::max_indices(state_actions);

#ifdef BITRL_DEBUG
    assert(best_actions.size() <= action_space_size_ &&
           "Incompatible number of best actions. Cannot exccedd the action space size");
#endif

    std::vector<std::pair<uint_t, real_t>> best_actions_vals(best_actions.size());

    for (uint_t i = 0; i < best_actions.size(); ++i)
    {
        best_actions_vals[i] = {best_actions[i], 1.0 / best_actions.size()};
    }

    auto &state_action_vals = this->policy_.state_actions_values();

    auto &view = state_action_vals[state];

    // collect all the actions in a map
    auto act_val_map = std::unordered_map<uint_t, real_t>();

    for (uint_t a = 0; a < best_actions_vals.size(); ++a)
    {
        act_val_map.insert({best_actions_vals[a].first, best_actions_vals[a].second});
    }

    for (uint_t a = 0; a < view.size(); ++a)
    {
        auto action = view[a].first;

        if (act_val_map.contains(action))
        {
            view[a].second = act_val_map[action];
        }
        else
        {
            view[a].second = 0.0;
        }
    }

    return this->policy_;
}

} // namespace policies

} // namespace rl

} // namespace bitrl

#endif // STOCHASTIC_ADAPTOR_POLICY_H
