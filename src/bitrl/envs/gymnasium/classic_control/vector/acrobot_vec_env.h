#ifndef ACROBOT_ENV_H
#define ACROBOT_ENV_H

/**
 *  Vector Acrobot environment. This class simply wraps copies of the 
 *  Acrobot class. 
 * See: https://github.com/pockerman/rlenvs_from_cpp/blob/master/src/rlenvs/envs/gymnasium/classic_control/acrobot_env.h
 * for more information
 **/

#include "bitrl/bitrl_types.h"
#include "bitrl/envs/vector_time_step.h"
#include "bitrl/envs/space_type.h"
#include "bitrl/envs/gymnasium/gymnasium_vector_env_base.h"
#include "../../../../network/rest_rl_env_client.h"
#include "bitrl/extern/nlohmann/json/json.hpp"


#include <string>
#include <vector>
#include <any>


namespace bitrl{

/// Forward declaration
template<typename StateTp> class VectorTimeStep;

namespace envs::gymnasium
{

	namespace detail_{

		struct AcrobotVEnv{

			typedef ContinuousVectorSpace<6, real_t> state_space;

			typedef state_space::space_item_type state_type;

			///
		/// \brief state space size
		///
			static constexpr uint_t STATE_SPACE_SIZE = state_space::size;

			typedef ScalarDiscreteSpace<0, 3> action_space;

			///
		/// \brief the Action type
		///
			typedef std::vector<typename action_space::space_item_type> action_type;

			///
		/// \brief state space size
		///
			static constexpr uint_t ACTION_SPACE_SIZE = action_space::size;

		};

	}

	///
/// \brief The CartPole class Interface for CartPole environment
///
	class AcrobotV final: public GymnasiumVecEnvBase<VectorTimeStep<detail_::AcrobotVEnv::state_type>,
	                                                 detail_::AcrobotVEnv>
	{

	public:

		///
    /// \brief name
    ///
		static  const std::string name;

		///
	/// \brief The URI for accessing the environment
	///
		static const std::string URI;

		///
	/// \brief Base class type
	///
		typedef GymnasiumVecEnvBase<VectorTimeStep<detail_::AcrobotVEnv::state_type>,
		                            detail_::AcrobotVEnv>::base_type base_type;

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
    /// \brief Acrobot. Constructor
    ///
		AcrobotV(const RESTRLEnvClient& api_server );

		///
    /// \brief CartPole. Constructor
    ///
		AcrobotV(const RESTRLEnvClient& api_server ,
		         const uint_t cidx);

		///
	/// \brief copy ctor
	///
		AcrobotV(const AcrobotV& other);

		///
    /// \brief ~Acrobot. Destructor
    ///
		~AcrobotV()=default;

		///
    /// \brief make. Build the environment
    ///
		virtual void make(const std::string& version,
		                  const std::unordered_map<std::string, std::any>&) override final;


		///
    /// \brief step. Step in the environment following the given action
    ///
		virtual time_step_type step(const action_type& action) override final;

		///
	/// \brief Create a new copy of the environment with the given
	/// copy index
	///
		AcrobotV make_copy(uint_t cidx)const;

		///
    /// \brief n_actions. Returns the number of actions
    ///
		uint_t n_actions()const noexcept{return action_space_type::size;}

	protected:

		///
    /// \brief Handle the reset response from the environment server
    ///
		virtual time_step_type create_time_step_from_response_(const nlohmann::json& response) const override final;

	};



}
}

#endif // PENDULUM_ENV_H
