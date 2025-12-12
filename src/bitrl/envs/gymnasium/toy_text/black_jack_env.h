#ifndef BLACK_JACK_H
#define BLACK_JACK_H

/**
 *  BlackJack environment
 *  https://github.com/Farama-Foundation/Gymnasium/blob/main/gymnasium/envs/toy_text/blackjack.py
 */

#include "bitrl/bitrl_types.h"
#include "bitrl/envs/time_step.h"
#include "bitrl/network/rest_rl_env_client.h"
#include "bitrl/envs/gymnasium/toy_text/toy_text_base.h"
#include "bitrl/extern/nlohmann/json/json.hpp"

#include <string>
#include <vector>
#include <tuple>
#include <any>
#include <unordered_map>


namespace bitrl{
namespace envs::gymnasium
{

	///
/// \brief The BlackJack class. Wrapper to the Blackjack
/// OpenAI-Gym environment.
///
class BlackJack final: public ToyTextEnvBase<TimeStep<uint_t>, 48, 2>
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
	/// \brief The base type
	///
		typedef typename ToyTextEnvBase<TimeStep<uint_t>, 48, 2>::base_type base_type;

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
    /// \brief BlackJack. Constructor.
    ///
		BlackJack(const RESTRLEnvClient& api_server);



		///
	///
	///
		BlackJack(const BlackJack& other);

		///
    /// \brief ~BlackJack. Destructor
    ///
		~BlackJack()=default;

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
		virtual time_step_type step(const action_type& action)override final;

		///
		///
		///
		bool is_natural()const noexcept{return is_natural_;}

		///
		///
		///
		bool is_sab()const noexcept{return is_sab_;}

protected:


		///
		/// \brief build the dynamics from response
		///
		virtual dynamics_t build_dynamics_from_response_(const nlohmann::json&)const override final;

		///
		/// \brief Handle the reset response from the environment server
		///
		virtual time_step_type create_time_step_from_response_(const nlohmann::json&) const override final;

private:

		///
    /// \brief Flag indicating if the environment has been
    /// initialized as natural
    ///
		bool is_natural_{false};
		bool is_sab_{false};

		void make_natural()noexcept{is_natural_ = true;}
		void make_sab()noexcept{is_sab_ = true;}

	};

}
}
#endif // BLACK_JACK_H
