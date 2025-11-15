#ifndef WEBOTS_BASE_ENV_H
#define WEBOTS_BASE_ENV_H

#include "rlenvs/rlenvscpp_config.h"

#ifdef RLENVSCPP_WEBOTS

#include "rlenvs/rlenvs_types_v2.h"
#include "rlenvs/envs/time_step.h"
#include "rlenvs/envs/env_base.h"


#include <memory>
#include <string>
#include <unordered_map>
#include <any>

namespace rlenvscpp{
namespace envs{
namespace webots_envs{

///
/// \brief Base class for creating environments for RL using
/// the Webots simulator. This class assumes that only one robot
/// exists in the world.
/// For a list of robots in webots checkout: https://www.cyberbotics.com/doc/guide/robots?version=R2019a-rev1
///
template<typename TimeStepType, typename SpaceType>
class WebotsEnvBase: public EnvBase<TimeStepType, SpaceType>{

public:
	
	///
	/// \brief the default time step for simulating the world
	/// the time step in milliseconds 
	/// Note that every robot device has to be initialised
	/// before being used. This is left to be done in the make
	/// function of concrete classes
	///
	static constexpr uint_t DEFAULT_SIM_TIME_STEP  = 32;
		
	///
	/// \brief Base class type
	///
	typedef EnvBase<TimeStepType, SpaceType> base_type;
									 
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
	
	///
	/// \brief pauses the webots simulation
	///
	virtual void pause_simulation() = 0;
	
protected:
	
	
	///
	/// \brief Constructor
	///
	WebotsEnvBase(const uint_t cidx, 
				  const std::string& name=rlenvscpp::consts::INVALID_STR);
	

}; 

template<typename TimeStepType, typename SpaceType>
WebotsEnvBase<TimeStepType, SpaceType>::WebotsEnvBase(const uint_t cidx, 
				                                      const std::string& name)
	:
	EnvBase<TimeStepType, SpaceType>(cidx, name)
{}
		
}
}
}

#endif 
#endif