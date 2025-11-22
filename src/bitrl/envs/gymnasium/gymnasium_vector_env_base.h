/**
  * Base class for Gymnasium vector environments.
  * See: https://gymnasium.farama.org/api/vector/sync_vector_env/
  * 
  *
*/


#ifndef GYMNASIUM_VECTOR_ENV_BASE_H
#define GYMNASIUM_VECTOR_ENV_BASE_H

#include "bitrl/bitrl_config.h"
#include "bitrl/envs/gymnasium/gymnasium_env_base.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/envs/env_types.h"
#include "bitrl/envs/api_server/apiserver.h"
#include "bitrl/extern/nlohmann/json/json.hpp"

#ifdef BITRL_DEBUG
#include <cassert>
#endif

#include <stdexcept>
	

namespace bitrl{
namespace envs{
namespace gymnasium {
	
	
template<typename VectorTimeStepType, typename SpaceType>
class GymnasiumVecEnvBase: public GymnasiumEnvBase<VectorTimeStepType, SpaceType>
{
	
public:
	
	///
	/// \brief The base type
	///
	typedef GymnasiumEnvBase<VectorTimeStepType, SpaceType>::base_type base_type;
	
	///
	/// \brief The time step type we return every time a step in the
	/// environment is performed
	///
    typedef typename base_type::time_step_type time_step_type;
	
	///
	/// \brief The type describing the state space for the environment
	///
	typedef typename base_type::state_space_type state_space_type;
	
	///
	/// \brief The type of the action space for the environment
	///
	typedef typename base_type::action_space_type action_space_type;

    ///
	/// \brief The type of the action to be undertaken in the environment
	///
    typedef typename base_type::action_type action_type;
	
	///
	/// \brief The type of the state
	///
	typedef typename base_type::state_type state_type;
	
	virtual void make(const std::string& version,
					  const std::unordered_map<std::string, std::any>& options)=0;
	
	
	///
	/// \brief Returns the number of environments
	///
	uint_t get_n_envs()const noexcept {return n_envs_;}
	
	///
	/// \brief 
	///
	void reset_if_any_done(bool flag)noexcept{reset_if_any_done_ = flag;}
	
	///
	/// \brief 
	///
	bool get_reset_if_any_done()const noexcept{return reset_if_any_done_;}
	
protected:
	
	///
    /// \brief Constructor
    ///
    GymnasiumVecEnvBase(const RESTApiServerWrapper& api_server,
						const uint_t cidx,
	                    const std::string& name);
						
	GymnasiumVecEnvBase(const GymnasiumVecEnvBase& other);
					  
private:
	
	
	uint_t n_envs_{0};
	
	///
	/// \brief Flag indicating that the vector environment
	/// is reset if any of the environments is done
	///
	bool reset_if_any_done_{false};
	
};


template<typename VectorTimeStepType, typename SpaceType>
GymnasiumVecEnvBase<VectorTimeStepType, SpaceType>::GymnasiumVecEnvBase(const RESTApiServerWrapper& api_server,
                                                                        const uint_t cidx,
															            const std::string& name)
:
GymnasiumEnvBase<VectorTimeStepType, SpaceType>(api_server, cidx, name)
{}


template<typename VectorTimeStepType, typename SpaceType>
GymnasiumVecEnvBase<VectorTimeStepType, SpaceType>::GymnasiumVecEnvBase(const GymnasiumVecEnvBase<VectorTimeStepType, SpaceType>& other)
:
GymnasiumEnvBase<VectorTimeStepType, SpaceType>(other),
n_envs_(other.n_envs_),
reset_if_any_done_(other.reset_if_any_done_)
{}

template<typename VectorTimeStepType, typename SpaceType>
void 
GymnasiumVecEnvBase<VectorTimeStepType, SpaceType>::make(const std::string& version,
                                             const std::unordered_map<std::string, std::any>& options){

	auto reset_if_any_done_itr = options.find("reset_if_any_done");
	
	if(reset_if_any_done_itr != options.end()){
		reset_if_any_done_ = std::any_cast<bool>(reset_if_any_done_itr->second);
	}
	
	auto n_envs_itr = options.find("num_envs");
	
	if(n_envs_itr != options.end()){
		n_envs_ = std::any_cast<uint_t>(n_envs_itr->second);
		return;
	}
	
	throw std::logic_error("num_envs variable is not provided");
												 
}
	
}
}
}


#endif // GYMNASIUMENVBASE_H
