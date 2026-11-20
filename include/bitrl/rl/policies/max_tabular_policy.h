#ifndef MAX_TABULAR_POLICY_H
#define MAX_TABULAR_POLICY_H

#include "bitrl/base/bitrl_config.h"
#include "bitrl/base/bitrl_types.h"
#include "bitrl/rl/algorithms/utils.h"
#include "bitrl/maths/vector_math.h"

#ifdef USE_PYTORCH
#include <torch/torch.h>
#endif

#include <type_traits>
#include <vector>
#include <string>
#include <iostream>

namespace bitrl {
namespace rl {
namespace policies {
	
/// Forward declaration. Definition
/// at the end of the file
struct MaxTabularPolicyBuilder;
	
///
/// \brief class MaxTabularPolicy
///
class MaxTabularPolicy
{
public:

    ///
	/// \brief The output type of operator()
	///
    typedef uint_t output_type;
	typedef uint_t state_type;
	typedef uint_t action_type;
	
	///
	/// \brief get_action. Given a
	///
    template<typename MatType>
    static output_type get_action(const MatType& q_map, uint_t state_idx);
	
	///
	/// \brief get_action. Given a vector always returns the position
	/// of the maximum occuring element. If the given vector is empty returns
	/// CubeAIConsts::invalid_size_type
	///
    template<typename VecTp>
    static output_type get_action(const VecTp& q_map);
	
#ifdef USE_PYTORCH
	///
	/// \brief get_action. Given a vector always returns the position
	/// of the maximum occuring element. If the given vector is empty returns
	/// CubeAIConsts::invalid_size_type
	///
    static output_type get_action(const torch_tensor_t& vec);
#endif

	/// 
	/// \brief Make friends so the builder access
	/// private members
	///
	friend struct MaxTabularPolicyBuilder;

    /// 
    /// \brief Constructor
    /// 
    MaxTabularPolicy()=default;

    /// 
	/// \brief any actions the policy should perform
	/// on the given episode index
	/// 
    void on_episode(uint_t)noexcept{}

    ///
	/// \brief Reset the policy
	///
    void reset()noexcept{state_action_map_.clear();}
	
	///
	/// \brief Get the action from the given state
	///
	action_type on_state(state_type s)const{return state_action_map_[s];}
	
	///
	/// \brief Save the state -> action map in a CSV file;
	///
	void save(const std::string& filename)const;
	
private:

		///
		/// \brief The state-action map
		///
		std::vector<uint_t> state_action_map_;
};


#ifdef USE_PYTORCH
inline
uint_t
MaxTabularPolicy::get_action(const torch_tensor_t& vec){
    return torch::argmax(vec).item<uint_t>();
}
#endif


template<typename VecTp>
MaxTabularPolicy::action_type
MaxTabularPolicy::get_action(const VecTp& vec){

    return std::distance(vec.begin(),
                         std::max_element(vec.begin(),
                                          vec.end()));

}


struct MaxTabularPolicyBuilder
{

	template<typename EnvType>
	void build_from_state_function(const EnvType& env,
                                   const DynVec<real_t>& v,
                                   real_t gamma, 
								   MaxTabularPolicy& policy);
								   
	void build_from_state_action_function(const DynMat<real_t>& q,
	                                      MaxTabularPolicy& policy);   
};

template<typename EnvType>
void
MaxTabularPolicyBuilder::build_from_state_function(const EnvType& env,
												   const DynVec<real_t>& v,
                                                   real_t gamma, 
								                   MaxTabularPolicy& policy){
																
	static_assert(std::is_integral_v<typename EnvType::state_type>, 
	              "state type must be integral");
	static_assert(std::is_integral_v<typename EnvType::action_type>, 
	              "action type must be integral");
								

	typedef typename EnvType::action_type action_type;
	policy.state_action_map_.clear();
	policy.state_action_map_.resize(env.n_states());

	for(uint_t s=0; s<env.n_states(); ++s){
	
		auto state_vals = bitrl::rl::algos::state_actions_from_v(env, v, 
																  gamma, s);
																  
		action_type action = policy.get_action(state_vals);
		policy.state_action_map_[s] = action;
	}
	
}


  



}
}
}

#endif // MAX_TABULAR_POLICY_H
