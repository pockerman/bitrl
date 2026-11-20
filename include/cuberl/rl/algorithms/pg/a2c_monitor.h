#ifndef A2C_MONITOR_H
#define A2C_MONITOR_H

#include "cuberl/base/cuberl_config.h"

#ifdef USE_PYTORCH

#include "cuberl/base/cuberl_types.h"
#include "cuberl/utils/experience_buffer.h"


#include <vector>
#include <tuple>

namespace cuberl {
namespace rl {
namespace algos {
namespace pg {
	
template<typename ActionType, typename StateType>
struct A2CMonitor
{
	
	typedef StateType state_type;
	typedef ActionType action_type;
	typedef std::tuple<state_type, // the state observed
	                   action_type,  // the action taken
	                   real_t, // the reward received
					   bool, // done?
					   torch_tensor_t, // log prob
					   torch_tensor_t  // critic values
					   > experience_tuple_type;

	typedef cuberl::utils::ExperienceBuffer<experience_tuple_type> experience_buffer_type;
	
	/// monitor 
    std::vector<real_t> rewards;
	std::vector<real_t> policy_loss_values;
	std::vector<real_t> critic_loss_values;
	std::vector<uint_t> episode_duration;
	

	void reset()noexcept;
	
	template<typename T, uint_t index>
    std::vector<T> 
    get(const std::vector<experience_tuple_type>& experience)const;
	
};


template<typename ActionType, typename StateType>
template<typename T, uint_t index>
std::vector<T> 
A2CMonitor<ActionType, StateType>::get(const std::vector<experience_tuple_type>& experience)const{
	
	std::vector<T> result;
	result.reserve(experience.size());
	
	auto b = experience.begin();
	auto e = experience.end();
	
	for(; b != e; ++b){
		auto item = *b;
		result.push_back(std::get<index>(item));
	}
	
	return result;
}

template<typename ActionType, typename StateType>
void 
A2CMonitor<ActionType, StateType>::reset()noexcept{

	policy_loss_values.clear();
	rewards.clear();
	episode_duration.clear();
}
	
}
}
}
}




#endif
#endif
