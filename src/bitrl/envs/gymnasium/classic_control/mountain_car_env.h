#ifndef MOUNTAIN_CAR_H
#define MOUNTAIN_CAR_H

#include "bitrl/bitrl_config.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/envs/gymnasium/gymnasium_env_base.h"
#include "bitrl/envs/time_step.h"
#include "bitrl/envs/env_types.h"
#include "bitrl/envs/api_server/apiserver.h"
#include "bitrl/extern/nlohmann/json/json.hpp"

#include <string>
#include <vector>
#include <any>
#include <unordered_map>

namespace bitrl{
namespace envs::gymnasium
{

	///
/// \brief The MountainCar class
///
	class MountainCar final:  public GymnasiumEnvBase<TimeStep<std::vector<real_t>>,
	                                                  ContinuousVectorStateDiscreteActionEnv<3, 2, 0, real_t >
		>
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
		typedef GymnasiumEnvBase<TimeStep<std::vector<real_t> >,
		                         ContinuousVectorStateDiscreteActionEnv< 3, // size of state
		                                                                 2, // end of action space
		                                                                 0, // start of action space
		                                                                 real_t> // type of state
		>::base_type base_type;
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
    /// \brief MountainCar. Constructor. Creates an environment.
    /// \param version The version of the environment
    /// \param gym_namespace The boost::python open-ai gym namespace
    /// \param do_create If true it calls make
    ///
		MountainCar(const RESTApiServerWrapper& api_server );

		///
	/// \brief Constructor. Protected so that applications
	/// cannot explicitly instantiate copies
	///
		MountainCar(const RESTApiServerWrapper& api_server ,
		            const uint_t cidx);

		///
	/// \brief copy ctor
	///
		MountainCar(const MountainCar& other);

		///
    /// \brief ~MountainCar. Destructor.
    ///
		~MountainCar()=default;

		///
    /// \brief make. Build the environment
    ///
		virtual void make(const std::string& version,
		                  const std::unordered_map<std::string, std::any>& /*options*/) override final;


		///
    /// \brief step
    ///
		virtual time_step_type step(const action_type& action)override final;

		///
	/// \brief Create a new copy of the environment with the given
	/// copy index
	///
		MountainCar make_copy(uint_t cidx)const;

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
#endif // MOUNTAIN_CAR_H
