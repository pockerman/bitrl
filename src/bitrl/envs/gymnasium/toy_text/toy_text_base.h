#ifndef TOY_TEXT_BASE_ENV_H
#define TOY_TEXT_BASE_ENV_H

#include "bitrl/envs/gymnasium/gymnasium_env_base.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/extern/nlohmann/json/json.hpp"
#include "bitrl/bitrl_config.h"
#include "bitrl/network/rest_rl_env_client.h"
#include "bitrl/envs/env_types.h"

#include <string>
#include <vector>
#include <tuple>
#include <unordered_map>
#include <any>

#ifdef BITRL_DEBUG
#include <cassert>
#endif

namespace bitrl{
namespace envs::gymnasium
{

///
/// \brief ToyTextEnvBase class. Base class
/// for toy environments from Gymnasium. These environments
/// have a discrete action and state spaces

template<typename TimeStepType, uint_t state_end,  uint_t action_end>
class ToyTextEnvBase: public GymnasiumEnvBase<TimeStepType,
											   ScalarDiscreteEnv<state_end, action_end, 0, 0> >{
public:

		///
	/// \brief The base_type
	///
	typedef typename GymnasiumEnvBase<TimeStepType,
		                                  ScalarDiscreteEnv<state_end, action_end, 0, 0> >::base_type base_type;

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
    /// \brief dynamics_t
    ///
	typedef std::vector<std::tuple<real_t, uint_t, real_t, bool>> dynamics_t;

	///
    /// \brief ~FrozenLake. Destructor.
    ///
	virtual ~ToyTextEnvBase()=default;

	///
    /// \brief P
    /// \param sidx
    /// \param aidx
    ///
	dynamics_t p(uint_t sidx, uint_t aidx)const;

	///
    /// \brief n_actions. Returns the number of actions
    ///
	uint_t n_actions()const noexcept{return action_space_type::size;}

	///
    /// \brief Number of states.
    ///
	uint_t n_states()const noexcept{return state_space_type::size;}


protected:

	///
    /// \brief Constructor
    ///
	ToyTextEnvBase(network::RESTRLEnvClient& api_server, const std::string& name);

	///
	/// \brief Copy constructor
	///
	ToyTextEnvBase(const ToyTextEnvBase& other);

	///
    /// \brief build the dynamics from response
    ///
	virtual dynamics_t build_dynamics_from_response_(const nlohmann::json& response)const;

};

template<typename TimeStepType, uint_t state_end,  uint_t action_end>
ToyTextEnvBase<TimeStepType, state_end, action_end>::ToyTextEnvBase(network::RESTRLEnvClient& api_server,const std::string& name)
		:
		GymnasiumEnvBase<TimeStepType,
		                 ScalarDiscreteEnv<state_end, action_end>>(api_server,
		                                                           name)
	{}

template<typename TimeStepType, uint_t state_end,  uint_t action_end>
ToyTextEnvBase<TimeStepType,
	               state_end,
	               action_end>::ToyTextEnvBase(const ToyTextEnvBase<TimeStepType,
	                                                                state_end, action_end>& other)
		:
		GymnasiumEnvBase<TimeStepType,
		                 ScalarDiscreteEnv<state_end, action_end>>(other)
	{}


	template<typename TimeStepType, uint_t state_end,  uint_t action_end>
	typename ToyTextEnvBase<TimeStepType, state_end, action_end>::dynamics_t
	ToyTextEnvBase<TimeStepType, state_end, action_end>::p(uint_t sidx, uint_t aidx)const{

#ifdef BITRL_DEBUG
		assert(this->is_created() && "Environment has not been created");
#endif

		auto response = this -> get_api_server().dynamics(this->env_name(),
		                                                  this->idx(),
		                                                  sidx, aidx);
		return build_dynamics_from_response_(response);
	}


	template<typename TimeStepType, uint_t state_end,  uint_t action_end>
	typename ToyTextEnvBase<TimeStepType, state_end, action_end>::dynamics_t
	ToyTextEnvBase<TimeStepType,
	               state_end,
	               action_end>::build_dynamics_from_response_(const nlohmann::json& response)const{
		auto dynamics = response["dynamics"];
		return dynamics;
	}


}
} // rlenvs_cpp
#endif
