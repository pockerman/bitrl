#ifndef RANDOM_TABULAR_POLICY_H
#define RANDOM_TABULAR_POLICY_H

#include "bitrl/base/bitrl_types.h"
#include "bitrl/base/bitrl_config.h"
#include "bitrl/utils/torch_adaptor.h"


#ifdef USE_PYTORCH
#include <torch/torch.h>
#endif

#include <random>

namespace bitrl {
namespace rl {
namespace policies {

/// 
/// \brief class RandomTabularPolicy
/// 
class RandomTabularPolicy
{

public:
	
	///
	/// \brief The type returned when calling this->operator()
	///
	typedef uint_t output_type;

    /// 
	/// \brief Constructor
	/// 
    RandomTabularPolicy();

	///
	/// \brief Constructor Initialize with a seed
	///
    explicit RandomTabularPolicy(uint_t seed);

	///
	/// \brief operator(). Given a
	///
    template<typename MatType>
    output_type operator()(const MatType& q_map, uint_t state_idx)const;

#ifdef USE_PYTORCH
    output_type operator()(const torch_tensor_t& vec)const;
#endif

    ///
	/// \brief operator(). Given a vector always returns the position
	/// of the maximum occuring element. If the given vector is empty returns
	/// CubeAIConsts::invalid_size_type
	///
    template<typename VecTp>
    output_type operator()(const VecTp& vec)const;
	
	
	///
	/// \brief get_action. Given a
	///
    template<typename MatType>
    output_type get_action(const MatType& q_map, uint_t state_idx);
	
	///
	/// \brief get_action. Given a vector always returns the position
	/// of the maximum occuring element. If the given vector is empty returns
	/// CubeAIConsts::invalid_size_type
	///
    template<typename VecTp>
    output_type get_action(const VecTp& q_map);

    ///
	/// \brief any actions the policy should perform
	/// on the given episode index
	///
    void on_episode(uint_t)noexcept{}

    ///
	/// \brief Reset the policy
	///
    void reset()noexcept{}

private:
	
	///
	/// \brief Random device to use when not using seed
	///
	//std::random_device rd_;

    ///
	/// \brief The random engine generator
	///
    mutable std::mt19937 generator_;
};

#ifdef USE_PYTORCH
inline
RandomTabularPolicy::output_type
RandomTabularPolicy::operator()(const torch_tensor_t& vec)const{

    auto vector = bitrl::utils::pytorch::TorchAdaptor::to_vector<real_t>(vec);
    //std::discrete_distribution<int> distribution(vector.begin(), vector.end());
	std::uniform_int_distribution<uint_t> distribution(0, vector.size()-1);
    return distribution(generator_);

}
#endif

template<typename VecTp>
RandomTabularPolicy::output_type
RandomTabularPolicy::operator()(const VecTp& vec)const{

    //std::discrete_distribution<int> distribution(vec.begin(), vec.end());
	std::uniform_int_distribution<uint_t> distribution(0, vec.size()-1);
    return distribution(generator_);

}

template<typename VecTp>
RandomTabularPolicy::output_type
RandomTabularPolicy::get_action(const VecTp& vec){
	std::uniform_int_distribution<uint_t> distribution(0, vec.size()-1);
    return distribution(generator_);
}

}
}
}

#endif // RANDOM_TABULAR_POLICY_H
