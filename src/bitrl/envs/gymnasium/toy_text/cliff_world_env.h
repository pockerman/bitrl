#ifndef CLIFF_WORLD_H
#define CLIFF_WORLD_H

/**
 *
 * This is a simple implementation of the Gridworld Cliff einforcement learning task.
 *
 *   ### Description
 *   The board is a 4x12 matrix, with (using NumPy matrix indexing):
 *
 *   - [3, 0] as the start at bottom-left
 *   - [3, 11] as the goal at bottom-right
 *   - [3, 1..10] as the cliff at bottom-center
 *
 *   If the agent steps on the cliff it returns to the start.
 *   An episode terminates when the agent reaches the goal.
 *
 * ### Actions
 *
 * There are 4 discrete deterministic actions:
 *   - 0: move up
 *   - 1: move right
 *   - 2: move down
 *   - 3: move left
 *
 *   ### Observations
 *
 *   There are 3x12 + 1 possible states. In fact, the agent cannot be at the cliff, nor at
 *   the goal (as this results the end of episode). They remain all the positions of the first 3 rows plus the bottom-left cell.
 *   The observation is simply the current position encoded as [flattened index](https://numpy.org/doc/stable/reference/generated/numpy.unravel_index.html).
 *
 *  ### Reward
 *
 *   Each time step incurs -1 reward, and stepping into the cliff incurs -100 reward.
 *
 */

#include "bitrl/bitrl_types.h"
#include "bitrl/envs/time_step.h"
#include "bitrl/envs/gymnasium/toy_text/toy_text_base.h"
#include "bitrl/network/rest_rl_env_client.h"
#include "bitrl/extern/nlohmann/json/json.hpp"


#include <string>
#include <tuple>
#include <any>
#include <unordered_map>


namespace bitrl{
namespace envs::gymnasium
{

	///
/// \brief The CliffWorld class
///
class CliffWorld final: public ToyTextEnvBase<TimeStep<uint_t>, 37, 4>
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
		/// \brief dynamics_t
		///
		typedef std::vector<std::tuple<real_t, uint_t, real_t, bool>> dynamics_type;

		///
		/// \brief The base type
		///
		typedef typename ToyTextEnvBase<TimeStep<uint_t>, 37, 4>::base_type base_type;

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
    /// \brief CliffWorld
    ///
		CliffWorld(network::RESTRLEnvClient& api_server);


		///
	/// \brief copy constructor
	///
		CliffWorld(const CliffWorld& other);

		///
    /// \brief ~CliffWorld. Destructor
    ///
		~CliffWorld()=default;

		///
    /// \brief make. Builds the environment. Optionally we can choose if the
    /// environment will be slippery
    ///
		virtual void make(const std::string& version,
		                  const std::unordered_map<std::string, std::any>& options,
		                  const std::unordered_map<std::string, std::any>& reset_options) override final;

		///
    /// \brief step
    /// \param action
    /// \return
    ///
		virtual time_step_type step(const action_type& action) override final;

	protected:

		///
	/// \brief Maximum episodes per step
	///
		uint_t max_episode_steps_;



		///
    /// \brief Handle the reset response from the environment server
    ///
		virtual time_step_type create_time_step_from_response_(const nlohmann::json& response) const override final;
	};

}
}
#endif // CLIFF_WORLD_H
