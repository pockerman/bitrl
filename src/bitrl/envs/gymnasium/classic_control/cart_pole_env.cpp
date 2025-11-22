#include "bitrl/envs/gymnasium/classic_control/cart_pole_env.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/envs/time_step.h"
#include "bitrl/envs/time_step_type.h"
#include "bitrl/bitrl_config.h"
#include "bitrl/extern/nlohmann/json/json.hpp"

#ifdef BITRL_DEBUG
#include <cassert>
#endif

namespace bitrl{
namespace envs::gymnasium
{

	const std::string CartPole::name = "CartPole";
	const std::string CartPole::URI = "/gymnasium/cart-pole-env";

	CartPole::time_step_type
	CartPole::create_time_step_from_response_(const nlohmann::json& response)const{

		auto step_type = response["time_step"]["step_type"].template get<uint_t>();
		auto reward    = response["time_step"]["reward"];
		auto discount  = response["time_step"]["discount"];
		auto obs       = response["time_step"]["observation"];
		auto info      = response["time_step"]["info"];
		return CartPole::time_step_type(TimeStepEnumUtils::time_step_type_from_int(step_type),
		                                reward, obs, discount,
		                                std::unordered_map<std::string, std::any>());
	}

	CartPole::CartPole(const RESTApiServerWrapper& api_server)
		:
		GymnasiumEnvBase<TimeStep<std::vector<real_t> >,
		                 ContinuousVectorStateDiscreteActionEnv<4, 2, 0, real_t >>(api_server, 0, CartPole::name)
	{
		this -> get_api_server().register_if_not(CartPole::name, CartPole::URI);
	}

	CartPole::CartPole(const RESTApiServerWrapper& api_server,
	                   const uint_t cidx)
		:
		GymnasiumEnvBase<TimeStep<std::vector<real_t> >,
		                 ContinuousVectorStateDiscreteActionEnv<4, 2, 0, real_t >>(api_server,
			cidx,
			CartPole::name)
	{
		this -> get_api_server().register_if_not(CartPole::name, CartPole::URI);
	}

	CartPole::CartPole(const CartPole& other)
		:
		GymnasiumEnvBase<TimeStep<std::vector<real_t> >,
		                 ContinuousVectorStateDiscreteActionEnv<4, 2, 0, real_t >>(other)
	{}

	void
	CartPole::make(const std::string& version,
	               const std::unordered_map<std::string, std::any>& options){

		if(this->is_created()){
			return;
		}


		nlohmann::json ops;
		auto has_rendering = options.find("render_mode");
		if(has_rendering != options.end()){
			auto render_str = std::any_cast<std::string>(has_rendering->second);
			ops["render_mode"] = render_str;
		}

		auto response  = this -> get_api_server().make(this -> env_name(),
		                                               this -> cidx(),
		                                               version, ops);

		this->set_version_(version);
		this->make_created_();
	}


	CartPole::time_step_type
	CartPole::step(const action_type& action){

#ifdef BITRL_DEBUG
		assert(this->is_created() && "Environment has not been created");
#endif

		if(this->get_current_time_step_().last()){
			return this->reset(42, std::unordered_map<std::string, std::any>());
		}

		const auto response  = this -> get_api_server().step(this -> env_name(),
		                                                     this -> cidx(),
		                                                     action);

		this->get_current_time_step_() = this->create_time_step_from_response_(response);
		return this->get_current_time_step_();
	}

	CartPole
	CartPole::make_copy(uint_t cidx)const{


		CartPole copy(this -> get_api_server(), cidx);
		std::unordered_map<std::string, std::any> ops;
		auto version = this -> version();
		copy.make(version, ops);
		return copy;
	}


}
}
