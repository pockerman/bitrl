#ifndef GYMNASIUM_ENV_BASE_H
#define GYMNASIUM_ENV_BASE_H

///
/// \file gymnasium/gymnasium_env_base.h
///


#include "bitrl/bitrl_types.h"
#include "bitrl/bitrl_config.h"
#include "bitrl/envs/env_base.h"
#include "../../network/rest_rl_env_client.h"
#include "bitrl/extern/nlohmann/json/json.hpp"
#include "bitrl/utils//std_map_utils.h"

#include <string>
#include <unordered_map>
#include <any>


#ifdef BITRL_DEBUG
#include <cassert>
#endif

namespace bitrl{
namespace envs::gymnasium
{
/**
	 * @class GymnasiumEnvBase
	 * @brief Base class for all Gymnasium environment wrappers.
	 *
	 * This template wraps a remote Gymnasium-compatible environment served through
	 * a REST API. It extends EnvBase and provides the common logic for environment
	 * reset, closing, and time-step handling via HTTP communication.
	 *
	 * @tparam TimeStepType  The type representing one interaction step with the environment.
	 * @tparam SpaceType     The type describing the observation and action spaces.
 */
template<typename TimeStepType, typename SpaceType>
class GymnasiumEnvBase: public EnvBase<TimeStepType, SpaceType>
{
public:

	/** @brief Base environment type alias. */
	typedef EnvBase<TimeStepType, SpaceType> base_type;

	/** @brief Time step returned at each environment step. */
	typedef typename base_type::time_step_type time_step_type;

	/** @brief Type describing the observation/state space of the environment. */
	typedef typename base_type::state_space_type state_space_type;

	/** @brief Type describing the action space of the environment. */
	typedef typename base_type::action_space_type action_space_type;

	/** @brief Type representing a valid action to execute. */
	typedef typename base_type::action_type action_type;

	/** @brief Type representing a state/observation returned by the environment. */
	typedef typename base_type::state_type state_type;

	/** @brief Import the reset() overloads from the base class. */
	using base_type::reset;

	/**
	 * @brief Virtual destructor.
	 */
	virtual ~GymnasiumEnvBase();

	/**
	* @brief Check whether the environment is still alive/connected.
	* @return True if the wrapper can still communicate with the server.
	*/
	virtual bool is_alive()const;

	/**
     * @brief Close the environment on the server and release any resources.
     */
	virtual void close() override;

	/**
	 * @brief Reset the environment to an initial state using the reset
	 * options specified during make.
	 *
	 * @return Initial time step after reset
	 */
	virtual time_step_type reset()override;

	/**
	 * @brief Retrieve the REST API wrapper instance used for communication.
	 * @return Read-only reference to the server wrapper.
	 */
	RESTRLEnvClient& get_api_server()const{return *api_server_;}

	/**
	 * @brief Get the full URL for this environment endpoint on the server.
	 * @return Environment URL string.
	 */
	std::string get_url()const;

protected:

	/**
	 * @brief Constructor.
	 *
	 * @param api_server  A reference to the REST server wrapper handling communication.
	 * @param cidx        Index of this environment instance within a simulation.
	 * @param name        Name of the environment.
	*/
	GymnasiumEnvBase(RESTRLEnvClient& api_server,
		                 const std::string& idx,
		                 const std::string& name);

	/**
	 * @brief Copy constructor.
	 */
	GymnasiumEnvBase(const GymnasiumEnvBase&);

	/**
	 * @brief Server wrapper handling communication with remote Gymnasium environment.
	 */
	RESTRLEnvClient* api_server_;

	/**
	 * @brief Build a TimeStepType instance from a server JSON response.
	 *
	 * Derived classes must parse the Gymnasium response and convert it into
	 * a fully constructed time step object.
	 *
	 * @param response  JSON payload returned by the server.
	 * @return Constructed time_step_type instance.
	 */
	virtual time_step_type create_time_step_from_response_(const nlohmann::json& response)const=0;

	};

	template<typename TimeStepType, typename SpaceType>
	GymnasiumEnvBase<TimeStepType,
	                 SpaceType>::GymnasiumEnvBase(RESTRLEnvClient& api_server,
	                                              const std::string& idx,
	                                              const std::string& name)
		:
		EnvBase<TimeStepType, SpaceType>(idx, name),
		api_server_(&api_server)
	{}

	template<typename TimeStepType, typename SpaceType>
	GymnasiumEnvBase<TimeStepType,
	                 SpaceType>::GymnasiumEnvBase(const GymnasiumEnvBase<TimeStepType, SpaceType>& other)
		:
		EnvBase<TimeStepType, SpaceType>(other),
		api_server_(other.api_server_)
	{}


	template<typename TimeStepType, typename SpaceType>
	GymnasiumEnvBase<TimeStepType, SpaceType>::~GymnasiumEnvBase(){

		try{
			close();
		}
		catch(...){

		}
	}

	template<typename TimeStepType, typename SpaceType>
	bool
	GymnasiumEnvBase<TimeStepType, SpaceType>::is_alive()const{
		auto response = this -> api_server_ -> is_alive(this->env_name(), this -> idx());
		return response["result"];
	}

template<typename TimeStepType, typename SpaceType>
void
GymnasiumEnvBase<TimeStepType, SpaceType>::close(){

	if(!this->is_created()){
		return;
	}

	auto response = this -> api_server_ -> close(this->env_name(),
		                                            this -> idx());
	this -> invalidate_is_created_flag_();
}

template<typename TimeStepType, typename SpaceType>
typename GymnasiumEnvBase<TimeStepType, SpaceType>::time_step_type
GymnasiumEnvBase<TimeStepType, SpaceType>::reset(){

	if(!this->is_created()){
#ifdef BITRL_DEBUG
			assert(this->is_created() && "Environment has not been created");
#endif
			return time_step_type();
	}

	auto& reset_ops = this -> reset_options();
	auto seed = utils::resolve<uint_t>("seed", reset_ops);

	auto response = this -> api_server_ -> reset(this->env_name(),
		                                          this -> idx(), seed,
		                                          nlohmann::json());

	this -> get_current_time_step_() = this->create_time_step_from_response_(response);
	return this -> get_current_time_step_();
}


template<typename TimeStepType, typename SpaceType>
std::string
GymnasiumEnvBase<TimeStepType, SpaceType>::get_url()const{
		return api_server_ -> get_env_url(this -> env_name());
}



}
} // rlenvs_cpp

#endif // GYMNASIUMENVBASE_H
