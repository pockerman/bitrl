#ifndef FROZEN_LAKE_H
#define FROZEN_LAKE_H

/**
  * Wrapper to the FrozenLake OpenAI-Gym environment.
  * The origina environment can be found at:
  * https://github.com/openai/gym/blob/master/gym/envs/toy_text/frozen_lake.py
  *  Frozen lake involves crossing a frozen lake from Start(S) to goal(G) without falling into any holes(H).
  * The agent may not always move in the intended direction due to the slippery nature of the frozen lake
  *
  *  The agent take a 1-element vector for actions.
  *  The action space is `(dir)`, where `dir` decides direction to move in which can be:
  *  - 0: LEFT
  *  - 1: DOWN
  *  - 2: RIGHT
  *  - 3: UP
  *
  * The observation is a value representing the agents current position as
  *      current_row * nrows + current_col
  *
  *  Reward schedule:
  *  - Reach goal(G): +1
  *  - Reach hole(H): 0
  *
  * ### Arguments
  *  ```
  *  gym.make('FrozenLake-v0', desc=None,map_name="4x4", is_slippery=True)
  *  ```
  *  `desc`: Used to specify custom map for frozen lake. For example,
  *      desc=["SFFF", "FHFH", "FFFH", "HFFG"].
  *  `map_name`: ID to use any of the preloaded maps.
  *      "4x4":[
  *          "SFFF",
  *          "FHFH",
  *          "FFFH",
  *          "HFFG"
  *          ]
  *      "8x8": [
  *          "SFFFFFFF",
  *          "FFFFFFFF",
  *          "FFFHFFFF",
  *          "FFFFFHFF",
  *          "FFFHFFFF",
  *          "FHHFFFHF",
  *          "FHFFHFHF",
  *          "FFFHFFFG",
  *      ]
  *  `is_slippery`: True/False. If True will move in intended direction with
  *  probability of 1/3 else will move in either perpendicular direction with
  *  equal probability of 1/3 in both directions.
  *      For example, if action is left and is_slippery is True, then:
  *      - P(move left)=1/3
  *      - P(move up)=1/3
  *      - P(move down)=1/3
  *
  * */

#include "bitrl/bitrl_types.h"
#include "bitrl/envs/time_step.h"
#include "bitrl/envs/gymnasium/toy_text/toy_text_base.h"
#include "bitrl/envs/api_server/apiserver.h"
#include "bitrl/extern/nlohmann/json/json.hpp"

#include <string>
#include <vector>
#include <tuple>
#include <any>
#include <unordered_map>

namespace bitrl{
namespace envs::gymnasium
{

	template<uint_t side_size>
	struct frozenlake_state_size;


	template<>
	struct frozenlake_state_size<4>
	{
		///
    /// \brief size
    ///
		static constexpr uint_t size = 16;

	};


	template<>
	struct frozenlake_state_size<8>
	{
		///
    /// \brief size
    ///
		static constexpr uint_t size = 64;

	};

	///
/// \brief The FrozenLake class. Wrapper to Gymnasium FrozenLake
/// environment
///
	template<uint_t side_size>
	class FrozenLake final: public ToyTextEnvBase<TimeStep<uint_t>,
	                                              frozenlake_state_size<side_size>::size,
	                                              3>
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
		typedef std::vector<std::tuple<real_t, uint_t, real_t, bool>> dynamics_t;


		///
	/// \brief The base type
	///
		typedef typename ToyTextEnvBase<TimeStep<uint_t>,
		                                frozenlake_state_size<side_size>::size,
		                                3>::base_type base_type;

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
    /// \brief Constructor.
    ///
		FrozenLake(const RESTApiServerWrapper& api_server);

		///
	/// \brief Constructor
	///
		FrozenLake(const RESTApiServerWrapper& api_server,
		           const uint_t cidx, bool slippery);

		///
	/// \brief copy constructor
	///
		FrozenLake(const FrozenLake& other);

		///
    /// \brief ~FrozenLake. Destructor.
    ///
		~FrozenLake()=default;

		///
    /// \brief make. Builds the environment. Optionally we can choose if the
    /// environment will be slippery
    ///
		virtual void make(const std::string& version,
		                  const std::unordered_map<std::string, std::any>& options) override final;

		///
	/// \brief Step in the environment following the given action
	///
		virtual time_step_type step(const action_type& action) override final;

		///
	/// \brief Create a new copy of the environment with the given
	/// copy index
	///
		FrozenLake<side_size> make_copy(uint_t cidx)const;


		///
    /// \brief map_type
    /// \return
    ///
		std::string map_type()const noexcept{return side_size == 4 ? "4x4" : "8x8";}

		///
    /// \brief is_slipery
    /// \return
    ///
		bool is_slippery()const noexcept{return is_slippery_;}


	protected:



		///
    /// \brief build the dynamics from response
    ///
		//virtual dynamics_t build_dynamics_from_response_(const nlohmann::json& response)const override final;

		///
    /// \brief Handle the reset response from the environment server
    ///
		virtual time_step_type create_time_step_from_response_(const nlohmann::json& response) const override final;

	private:

		///
    /// \brief is_slipery_
    ///
		bool is_slippery_;

	};

}
}
#endif // FROZEN_LAKE_H
