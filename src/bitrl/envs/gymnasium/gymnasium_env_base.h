#ifndef GYMNASIUM_ENV_BASE_H
#define GYMNASIUM_ENV_BASE_H

///
/// \file gymnasium/gymnasium_env_base.h
///


#include "bitrl/bitrl_types_v2.h"
#include "bitrl/rlenvscpp_config.h"
#include "bitrl/extern/nlohmann/json/json.hpp"
#include "bitrl/envs/env_base.h"
#include "bitrl/envs/api_server/apiserver.h"

#include <string>
#include <unordered_map>
#include <any>


#ifdef BITRL_DEBUG
#include <cassert>
#endif

namespace bitrl{
namespace envs::gymnasium
{

	///
	/// \brief class GymnasiumEnvBase. Base class for all Gymnasium wrappers
	/// The class has two parameters:
	///
	/// TimeStepType: the type of the time step to use
	///
	/// SpaceType: The type of the space-action space
	///
	template<typename TimeStepType, typename SpaceType>
	class GymnasiumEnvBase: public EnvBase<TimeStepType, SpaceType>{
	public:


		///
	/// \brief The base_type
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
	/// \brief The type of the action to be undertaken in the environment
	///
		typedef typename base_type::state_type state_type;

		///
	/// \brief Expose the various reset methods we use from base class
	///
		using base_type::reset;

		///
    /// \brief ~GymnasiumEnvBase. Destructor.
    ///
		virtual ~GymnasiumEnvBase();

		///
		/// \brief
		///
		virtual bool is_alive()const;

		///
		/// \brief close the environment
		///
		virtual void close() override;

		///
		/// \brief Reset the environment
		///
		virtual time_step_type reset(uint_t seed,
		                             const std::unordered_map<std::string, std::any>& options)override;

		///
		/// \brief Returns read reference to the underlying API server wrapper
		///
		const RESTApiServerWrapper& get_api_server()const{return api_server_;}

		///
		/// \brief Returns the full path on the server for this environment
		///
		std::string get_url()const;

	protected:

		///
		/// \brief Constructor
		///
		GymnasiumEnvBase(const RESTApiServerWrapper& api_server,
		                 const uint_t cidx,
		                 const std::string& name);

		///
		/// \brief Copy constructor
		///
		GymnasiumEnvBase(const GymnasiumEnvBase&);

		///
		/// \brief Pointer to the api server that handles the requests
		///
		RESTApiServerWrapper api_server_;


		///
	/// \brief read reference to the api server instance
	///
		RESTApiServerWrapper& get_api_server(){return api_server_;}

		///
    /// \brief build the time step from the server response
    ///
		virtual time_step_type create_time_step_from_response_(const nlohmann::json& response)const=0;

	};

	template<typename TimeStepType, typename SpaceType>
	GymnasiumEnvBase<TimeStepType,
	                 SpaceType>::GymnasiumEnvBase(const RESTApiServerWrapper& api_server,
	                                              const uint_t cidx,
	                                              const std::string& name)
		:
		EnvBase<TimeStepType, SpaceType>(cidx, name),
		api_server_(api_server)
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
		auto response = this -> api_server_.is_alive(this->env_name(), this -> cidx());
		return response["result"];
	}

	template<typename TimeStepType, typename SpaceType>
	void
	GymnasiumEnvBase<TimeStepType, SpaceType>::close(){

		if(!this->is_created()){
			return;
		}

		auto response = this -> api_server_.close(this->env_name(), this -> cidx());
		this -> invalidate_is_created_flag_();

	}

	template<typename TimeStepType, typename SpaceType>
	typename GymnasiumEnvBase<TimeStepType, SpaceType>::time_step_type
	GymnasiumEnvBase<TimeStepType, SpaceType>::reset(uint_t seed,
	                                                 const std::unordered_map<std::string, std::any>& /*options*/){

		if(!this->is_created()){
#ifdef BITRL_DEBUG
			assert(this->is_created() && "Environment has not been created");
#endif
			return time_step_type();
		}

		auto response = this -> api_server_.reset(this->env_name(),
		                                          this -> cidx(), seed,
		                                          nlohmann::json());

		this -> get_current_time_step_() = this->create_time_step_from_response_(response);
		return this -> get_current_time_step_();
	}


	template<typename TimeStepType, typename SpaceType>
	std::string
	GymnasiumEnvBase<TimeStepType, SpaceType>::get_url()const{
		return api_server_.get_env_url(this -> env_name());
	}



}
} // rlenvs_cpp

#endif // GYMNASIUMENVBASE_H
