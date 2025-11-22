#ifndef VECTOR_TIME_STEP_H
#define VECTOR_TIME_STEP_H


#include "bitrl/bitrl_types.h"
#include "bitrl/envs/time_step_type.h"
#include "bitrl/extern/nlohmann/json/json.hpp"

#include <vector>
#include <algorithm>
#include <any>
#include <unordered_map>
#include <ostream>

namespace bitrl{


///
/// \brief VectorTimeSetp class
///
template<typename StateType>
class VectorTimeStep
{

public:
	
	///
    /// \brief state_ Type of the state
    ///
    typedef StateType state_type;

	///
	/// \brief Default construcotr
	///
    VectorTimeStep()=default;
	
	
	///
    /// \brief VectorTimeStep. Constructor
    ///
    VectorTimeStep(const std::vector<TimeStepTp>& types, 
	               const std::vector<real_t>& rewards, 
				   const std::vector<state_type>&  obs);

    
	///
    /// \brief VectorTimeStep. Constructor
    ///
    VectorTimeStep(const std::vector<TimeStepTp>& types, 
	               const std::vector<real_t>& rewards, 
				   const std::vector<state_type>&  obs, 
				   const std::vector<real_t>& discount_factors);
				   
				   
	VectorTimeStep(const std::vector<TimeStepTp>& types, 
	               const std::vector<real_t>& rewards, 
				   const std::vector<state_type>&  obs, 
				   const std::vector<real_t>& discount_factors,
				   std::unordered_map<std::string, std::any>&& extra);
				   
				   
	///
    /// \brief TimeStep
    /// \param other
    ///
    VectorTimeStep(const VectorTimeStep& other);

    ///
	/// \brief Assignment operator
	///
    VectorTimeStep& operator=(const VectorTimeStep& other);

    ///
    /// \brief TimeStep
    /// \param other
    ///
    VectorTimeStep(VectorTimeStep&& other)noexcept;

    ///
    /// \brief operator =
    /// \param other
    /// \return
    ///
    VectorTimeStep& operator=(VectorTimeStep&& other)noexcept;

    ///
    /// \brief type
    /// \return
    ///
    const std::vector<TimeStepTp>& types()const noexcept{return types_;}

    ///
    /// \brief observation
    /// \return
    ///
    const std::vector<state_type>& observations()const{return obs_;}

    ///
    /// \brief reward
    /// \return
    ///
    const std::vector<real_t>& rewards()const noexcept{return rewards_;}
	
	///
	/// \brief Returns the sum of the rewards received
	///
	real_t reward()const noexcept;
	
	///
	/// \brief Returns true if any time step is LAST
	///
	bool done()const noexcept; 

    ///
    /// \brief discount. Returns the discount factor
    ///
    const std::vector<real_t>& discounts()const noexcept{return discounts_;}
	
	///
    /// \brief last
    /// \return
    ///
    bool last()const noexcept;

private:

	///
	/// \brief the step types for the environments
	///
	std::vector<TimeStepTp> types_;
	
	///
	/// \brief The rewards received for every environment
	///
	std::vector<real_t> rewards_;
	
	///
	/// \brief Observations
	///
	std::vector<state_type>  obs_;
	
	///
	/// \brief The discount factors for every environment
	///
	std::vector<real_t> discounts_;
	
	
	///
    /// \brief extra_
    ///
    std::unordered_map<std::string, std::any> extra_;

};

template<typename StateType>
VectorTimeStep<StateType>::VectorTimeStep(const std::vector<TimeStepTp>& types, 
	               const std::vector<real_t>& rewards, 
				   const std::vector<state_type>&  obs, 
				   const std::vector<real_t>& discount_factors,
				   std::unordered_map<std::string, std::any>&& extra)
		:
		types_(types),
		rewards_(rewards),
		obs_(obs),
		discounts_(discount_factors),
		extra_(extra)
		{}

template<typename StateType>
VectorTimeStep<StateType>::VectorTimeStep(const std::vector<TimeStepTp>& types, 
	                                      const std::vector<real_t>& rewards, 
				                          const std::vector<state_type>&  obs, 
				                          const std::vector<real_t>& discount_factors)
		:
		types_(types),
		rewards_(rewards),
		obs_(obs),
		discounts_(discount_factors)
		{}

template<typename StateType>
VectorTimeStep<StateType>::VectorTimeStep(const std::vector<TimeStepTp>& types, 
	                                      const std::vector<real_t>& rewards, 
				                          const std::vector<state_type>&  obs)
		:
		VectorTimeStep<StateType>(types, rewards, 
								  obs, std::vector<real_t>())
{}



template<typename StateType>
VectorTimeStep<StateType>::VectorTimeStep(const VectorTimeStep<StateType>& other)
    :
      types_(other.types_),
      rewards_(other.rewards_),
      obs_(other.obs_),
      discounts_(other.discounts_),
      extra_(other.extra_)
{}

template<typename StateType>
VectorTimeStep<StateType>&
VectorTimeStep<StateType>::operator=(const VectorTimeStep<StateType>& other){

    types_ = other.types_;
    rewards_ = other.rewards_;
    obs_ = other.obs_;
    discounts_ = other.discounts_;
    extra_ = other.extra_;
    return *this;
}

template<typename StateType>
VectorTimeStep<StateType>::VectorTimeStep(VectorTimeStep<StateType>&& other)noexcept
    :
      types_(other.types_),
      rewards_(other.rewards_),
      obs_(other.obs_),
      discounts_(other.discounts_),
      extra_(other.extra_)
{
    //other.clear();
}

template<typename StateType>
VectorTimeStep<StateType>&
VectorTimeStep<StateType>::operator=(VectorTimeStep<StateType>&& other)noexcept{

    types_ = other.types_;
    rewards_ = other.rewards_;
    obs_ = other.obs_;
    discounts_ = other.discounts_;
    extra_ = other.extra_;
    //other.clear();
    return *this;
}

template<typename StateType>
real_t 
VectorTimeStep<StateType>::reward()const noexcept{
	
	auto sum_ = 0.0;
	sum_ = std::accumulate(rewards_.begin(), rewards_.end(), sum_);
	return sum_;
}

template<typename StateType>
bool 
VectorTimeStep<StateType>::done()const noexcept{
	auto done_ = false;
	
	for(auto step_type: types_){
		if(step_type == TimeStepTp::LAST){
			done_ = true;
			break;
		}
	}
	
	return done_;
}


template<typename StateType>
bool 
VectorTimeStep<StateType>::last()const noexcept{
	return done();
}


template<typename StateTp>
inline
std::ostream& operator<<(std::ostream& out, const VectorTimeStep<StateTp>& step){

	using json = nlohmann::json;
	json j;
	
	auto types = step.types();
	std::vector<std::string> step_to_str(types.size());
	
	for(uint_t i =0; i<step_to_str.size(); ++i){
		step_to_str[i] = TimeStepEnumUtils::to_string(types[i]);
	}
	
	j["step_types"] = step_to_str;
	j["rewards"] = step.rewards();
	j["observations"] = step.observations();
	j["discounts"] = step.discounts();
	
    out<< j <<std::endl;
    return out;
}




}

#endif // VECTOR_TIME_STEP_H
