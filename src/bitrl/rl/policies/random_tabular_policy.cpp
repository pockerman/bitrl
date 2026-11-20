#include "bitrl/rl/policies/random_tabular_policy.h"
#include "bitrl/base/bitrl_types.h"
#include "bitrl/base/bitrl_config.h"

#ifdef BITRL_DEBUG
#include <cassert>
#endif

#include <vector>

namespace bitrl {
namespace rl {
namespace policies {

	
RandomTabularPolicy::RandomTabularPolicy()
:
generator_(std::random_device()())
{}	

RandomTabularPolicy::RandomTabularPolicy(uint_t seed)
:
generator_(seed)
{}



template<>
RandomTabularPolicy::output_type
RandomTabularPolicy::operator()(const std::vector<std::vector<real_t>>& mat,
                                uint_t state_idx)const{
#ifdef BITRL_DEBUG
    assert(state_idx < mat.size() && "Invalid state index. Should be state_idx < q_map.size()");
#endif

    return (*this)(mat[state_idx]);

}

template<>
RandomTabularPolicy::output_type
RandomTabularPolicy::operator()(const DynMat<real_t>& mat,
                                uint_t state_idx)const{
#ifdef BITRL_DEBUG
    assert(state_idx < mat.size() && "Invalid state index. Should be state_idx < q_map.size()");
#endif

    return (*this)(mat.row(state_idx));

}


template<>
RandomTabularPolicy::output_type 
RandomTabularPolicy::get_action(const std::vector<std::vector<real_t>>& mat, 
                                uint_t state_idx){
	
#ifdef BITRL_DEBUG
    assert(state_idx < mat.size() && "Invalid state index. Should be state_idx < q_map.size()");
#endif

	return get_action(mat[state_idx]);
}

template<>
RandomTabularPolicy::output_type 
RandomTabularPolicy::get_action(const DynMat<real_t>& mat, uint_t state_idx){
	
#ifdef BITRL_DEBUG
    assert(state_idx < mat.size() && "Invalid state index. Should be state_idx < q_map.size()");
#endif

	return get_action(mat.row(state_idx));
	
	
	
}

}
}
}
