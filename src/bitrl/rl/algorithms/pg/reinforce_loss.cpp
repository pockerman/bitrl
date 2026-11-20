#ifndef REINFORCE_LOSS_H
#define REINFORCE_LOSS_H

#include "bitrl/base/bitrl_config.h"

#ifdef USE_PYTORCH

#include "bitrl/base/bitrl_types.h"
#include "bitrl/maths/vector_math.h"
#include <vector>
#include <algorithm>

namespace bitrl {
namespace rl {
namespace algos {
namespace pg {
	
///
/// \brief compute_loss_item. Compute the product -rewards[i] * log_probs[i]
///
std::vector<torch_tensor_t>
compute_loss_item(const std::vector<real_t>& rewards, 
                  const std::vector<torch_tensor_t>& log_probs){
					  
	
	std::vector<torch_tensor_t> loss_items;
	loss_items.reserve(rewards.size());

	auto rewards_begin = rewards.begin();
	auto rewards_end = rewards.end();
	auto log_probs_begin = log_probs.begin();
	
	for(; rewards_begin != rewards_end; ++rewards_begin, ++log_probs_begin){
		
		auto loss_val = (*rewards_begin) * (*log_probs_begin);
		loss_items.push_back(-loss_val);
	}
					  
	return loss_items;
}


std::vector<real_t> 
compute_baseline_with_constant(const std::vector<real_t>& rewards,
							   real_t constant){

	std::vector<real_t> result;
	result.reserve(rewards.size());
	
	static auto write_val = [&result, constant](const real_t& val){
		result.push_back(val + constant);
	};
	
	std::for_each(rewards.begin(),
				 rewards.end(),
				 write_val);
	return result;
								   
}

std::vector<real_t> 
compute_baseline_with_mean(const std::vector<real_t>& rewards){
	
	using bitrl::maths::mean;
	auto mean_val = mean(rewards);
	return compute_baseline_with_constant(rewards, -mean_val);
}

std::vector<real_t> 
compute_baseline_with_standardization(const std::vector<real_t>& rewards,
                                      real_t eps){
	return bitrl::maths::standardize(rewards, eps);
}
	
	
}
}
}
}


#endif
#endif