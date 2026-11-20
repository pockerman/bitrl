#include "bitrl/rl/policies/epsilon_greedy_policy.h"

#include "bitrl/base/bitrl_types.h"
#include "bitrl/base/bitrl_config.h"

#ifdef BITRL_DEBUG
#include <cassert>
#include <boost/log/trivial.hpp>
#endif

#include <vector>

namespace bitrl {
namespace rl {
namespace policies {
	
	
#ifdef USE_PYTORCH
    EpsilonGreedyPolicy::output_type 
	EpsilonGreedyPolicy::operator()(const torch_tensor_t& tensor, torch_tensor_value_type<real_t>)const{
		
		auto vec = bitrl::utils::pytorch::TorchAdaptor::to_vector<real_t>(tensor);

		std::uniform_real_distribution<> real_dist_(0.0, 1.0);

		if(real_dist_(generator_) > eps_){
			// select greedy action with probability 1 - epsilon
			return max_policy_.get_action(vec);
		}

		// else select a random action
		return random_policy_(vec);
		
	}
	EpsilonGreedyPolicy::output_type 
	EpsilonGreedyPolicy::operator()(const torch_tensor_t& tensor, torch_tensor_value_type<float_t>)const{
		
		auto vec = bitrl::utils::pytorch::TorchAdaptor::to_vector<float_t>(tensor);

		std::uniform_real_distribution<> real_dist_(0.0, 1.0);

		if(real_dist_(generator_) > eps_){
			// select greedy action with probability 1 - epsilon
			return max_policy_.get_action(vec);
		}

		// else select a random action
		return random_policy_(vec);
		
	}
	EpsilonGreedyPolicy::output_type 
	EpsilonGreedyPolicy::operator()(const torch_tensor_t& tensor, torch_tensor_value_type<int_t>)const{
		
		auto vec = bitrl::utils::pytorch::TorchAdaptor::to_vector<int_t>(tensor);

		std::uniform_real_distribution<> real_dist_(0.0, 1.0);

		if(real_dist_(generator_) > eps_){
			// select greedy action with probability 1 - epsilon
			return max_policy_.get_action(vec);
		}

		// else select a random action
		return random_policy_(vec);
		
	}
	EpsilonGreedyPolicy::output_type 
	EpsilonGreedyPolicy::operator()(const torch_tensor_t& tensor, 
									torch_tensor_value_type<lint_t>)const{
		auto vec = bitrl::utils::pytorch::TorchAdaptor::to_vector<lint_t>(tensor);

		std::uniform_real_distribution<> real_dist_(0.0, 1.0);

		if(real_dist_(generator_) > eps_){
			// select greedy action with probability 1 - epsilon
			return max_policy_.get_action(vec);
		}

		// else select a random action
		return random_policy_(vec);
	}
#endif


template<>
EpsilonGreedyPolicy::output_type
EpsilonGreedyPolicy::operator()(const DynMat<real_t>& mat, uint_t state_idx)const{

#ifdef BITRL_DEBUG
    assert(state_idx < mat.size() && "Invalid state index. Should be state_idx < q_map.size()");
#endif

    return (*this)(mat.row(state_idx));
}

template<>
EpsilonGreedyPolicy::output_type
EpsilonGreedyPolicy::operator()(const std::vector<std::vector<real_t>>& mat,
                                uint_t state_idx)const{
#ifdef BITRL_DEBUG
    assert(state_idx < mat.size() && "Invalid state index. Should be state_idx < q_map.size()");
#endif

    return (*this)(mat[state_idx]);

}


template<>
EpsilonGreedyPolicy::output_type 
EpsilonGreedyPolicy::get_action(const std::vector<std::vector<real_t>>& mat,
                                uint_t state_idx){
	
#ifdef BITRL_DEBUG
    assert(state_idx < mat.size() && "Invalid state index. Should be state_idx < q_map.size()");
#endif

    return get_action(mat[state_idx]);
}

template<>
EpsilonGreedyPolicy::output_type 
EpsilonGreedyPolicy::get_action(const DynMat<real_t>& mat, uint_t state_idx){
	
#ifdef BITRL_DEBUG
    assert(state_idx < mat.size() && "Invalid state index. Should be state_idx < q_map.size()");
#endif

    return get_action(mat.row(state_idx));
}

void 
EpsilonGreedyPolicy::set_eps_value(real_t eps){
	
	if(decay_op_ != EpsilonDecayOption::NONE){
#ifdef BITRL_DEBUG
	BOOST_LOG_TRIVIAL(warning)<<"Epsilon is not update as you have set up a decay policy..."<<std::endl;
#endif
		return;
	}
	
	// need to guard against zero etc
	if(eps < min_eps_){
		eps_ = min_eps_;
	}
	else if(eps > max_eps_){
		eps_ = max_eps_;
	}
	else{
		eps_ = eps;
	}
}

void
EpsilonGreedyPolicy::on_episode(uint_t episode)noexcept{

    if(decay_op_ == EpsilonDecayOption::NONE)
        return;

    if(decay_op_  == EpsilonDecayOption::INVERSE_STEP ){

        if(episode == 0){
            episode = 1;
        }

        // there are various methods to do epsilon
        // reduction
        eps_ = 1.0 / episode;

        if(eps_ < min_eps_){
            eps_ = min_eps_;
        }
    }
    else if(decay_op_ == EpsilonDecayOption::EXPONENTIAL){
        eps_ = min_eps_ + (max_eps_ - min_eps_)*std::exp(-epsilon_decay_ * episode);
    }
    else if(decay_op_ == EpsilonDecayOption::CONSTANT_RATE){

        eps_ -= epsilon_decay_;

        if(eps_ < min_eps_){
            eps_ = min_eps_;
        }

    }
}

}
}
}
