#ifndef PENDULUM_ENV_H
#define PENDULUM_ENV_H

/**
  *
  * Pendulum environment. The original environment
  *  is described here: https://github.com/openai/gym/blob/master/gym/envs/classic_control/pendulum.py
  *  The state variables are:
  *
  *      ### Description

    The inverted pendulum swingup problem is based on the classic problem in control theory.
    The system consists of a pendulum attached at one end to a fixed point, and the other end being free.
    The pendulum starts in a random position and the goal is to apply torque on the free end to swing it
    into an upright position, with its center of gravity right above the fixed point.

    The diagram below specifies the coordinate system used for the implementation of the pendulum's
    dynamic equations.

    ![Pendulum Coordinate System](./diagrams/pendulum.png)

    -  `x-y`: cartesian coordinates of the pendulum's end in meters.
    - `theta` : angle in radians.
    - `tau`: torque in `N m`. Defined as positive _counter-clockwise_.

    ### Action Space

    The action is a `ndarray` with shape `(1,)` representing the torque applied to free end of the pendulum.

    | Num | Action | Min  | Max |
    |-----|--------|------|-----|
    | 0   | Torque | -2.0 | 2.0 |


    ### Observation Space

    The observation is a `ndarray` with shape `(3,)` representing the x-y coordinates of the pendulum's free
    end and its angular velocity.

    | Num | Observation      | Min  | Max |
    |-----|------------------|------|-----|
    | 0   | x = cos(theta)   | -1.0 | 1.0 |
    | 1   | y = sin(theta)   | -1.0 | 1.0 |
    | 2   | Angular Velocity | -8.0 | 8.0 |

    ### Rewards

    The reward function is defined as:

    *r = -(theta<sup>2</sup> + 0.1 * theta_dt<sup>2</sup> + 0.001 * torque<sup>2</sup>)*

    where `$\theta$` is the pendulum's angle normalized between *[-pi, pi]* (with 0 being in the upright position).
    Based on the above equation, the minimum reward that can be obtained is
    *-(pi<sup>2</sup> + 0.1 * 8<sup>2</sup> + 0.001 * 2<sup>2</sup>) = -16.2736044*,
    while the maximum reward is zero (pendulum is upright with zero velocity and no torque applied).

    ### Starting State

    The starting state is a random angle in *[-pi, pi]* and a random angular velocity in *[-1,1]*.

    ### Episode Truncation

    The episode truncates at 200 time steps.

    ### Arguments

    - `g`: acceleration of gravity measured in *(m s<sup>-2</sup>)* used to calculate the pendulum dynamics.
      The default value is g = 10.0 .

    ```
    gym.make('Pendulum-v1', g=9.81)
    ```

    ### Version History

    * v1: Simplify the math equations, no difference in behavior.
    * v0: Initial versions release (1.0.0).
  **/


#include "bitrl/bitrl_types.h"
#include "bitrl/envs/time_step.h"
#include "bitrl/envs/gymnasium/gymnasium_env_base.h"
#include "../../../network/rest_rl_env_client.h"
#include "bitrl/extern/nlohmann/json/json.hpp"
#include "bitrl/envs/env_types.h"

#include <string>
#include <vector>
#include <tuple>
#include <any>

namespace bitrl{
namespace envs::gymnasium
{

	///
	/// \brief The Pendulum class. Interface for Pendulum environment
	///
	class Pendulum final: public GymnasiumEnvBase<TimeStep<std::vector<real_t>>,
	                                              ContinuousVectorStateContinuousScalarBoundedActionEnv<3,
		                                              1,
		                                              RealRange<-2.0, 2.0>,
		                                              0, real_t>
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
		typedef GymnasiumEnvBase<TimeStep<std::vector<real_t>>,
		                         ContinuousVectorStateContinuousScalarBoundedActionEnv<3,
			                         1,
			                         RealRange<-2.0, 2.0>,
			                         0, real_t>
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
		/// \brief Pendulum. Constructor
		///
		Pendulum(const RESTRLEnvClient& api_server );

		///
		/// \brief Constructor
		///
		Pendulum(const RESTRLEnvClient& api_server,
		         const uint_t cidx);

		///
		/// \brief copy ctor
		///
		Pendulum(const Pendulum& other);

		///
		/// \brief ~Pendulum. Destructor
		///
		~Pendulum()=default;

		///
		/// \brief make. Build the environment
		///
		virtual void make(const std::string& version,
		                  const std::unordered_map<std::string, std::any>& /*options*/) override final;

		///
		/// \brief step. Step in the environment following the given action
		///
		virtual time_step_type step(const action_type& action)override final;

		///
		/// \brief Create a new copy of the environment with the given
		/// copy index
		///
		Pendulum make_copy(uint_t cidx)const;

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
