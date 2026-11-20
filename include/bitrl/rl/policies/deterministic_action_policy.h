#ifndef DETERMINISTIC_ACTION_POLICY_H
#define DETERMINISTIC_ACTION_POLICY_H

#include "cubeai/utils/cubeai_concepts.h"
#include <type_traits>

namespace cubeai{
namespace rl {
namespace policies{

///
/// \brief class DeterministicActionPolicy. a deterministic action apolicy
/// always selects the action indicated
///
template<utils::concepts::float_or_integral_vector PolicyValuesType, typename StateType>
requires(std::is_integral<StateType>::value)
class DeterministicActionPolicy
{
public:

    typedef PolicyValuesType policy_values_type;
    typedef typename policy_values_type::value_type action_type;
    typedef StateType state_type;

    ///
    /// \brief DeterministicActionPolicy
    /// \param values
    ///
    explicit DeterministicActionPolicy(policy_values_type&& values);

    ///
    ///
    ///
    const action_type& on_state(const state_type& state)const{return policy_values_[state];}

private:

    policy_values_type policy_values_;

};

template<utils::concepts::float_or_integral_vector PolicyValuesType, typename StateType>
requires(std::is_integral<StateType>::value)
DeterministicActionPolicy<PolicyValuesType, StateType>::DeterministicActionPolicy(policy_values_type&& values)
    :
    policy_values_(values)
{}



}

}
}

#endif // DETERMINISTIC_ACTION_POLICY_H
