#include "bitrl/envs/gymnasium/toy_text/cliff_world_env.h"
#include "bitrl/bitrl_config.h"
#include "bitrl/envs/time_step_type.h"
#include "bitrl/extern/nlohmann/json/json.hpp"

#ifdef BITRL_DEBUG
#include <cassert>
#endif

#include <iostream>
#include <any>
#include <memory>


namespace bitrl{
namespace envs::gymnasium
{

	const std::string CliffWorld::name = "CliffWalking";
	const std::string CliffWorld::URI = "/gymnasium/cliff-walking-env";

	CliffWorld::time_step_type
	CliffWorld::create_time_step_from_response_(const nlohmann::json& response)const{

		auto step_type = response["time_step"]["step_type"].template get<uint_t>();
		auto reward = response["time_step"]["reward"];
		auto discount = response["time_step"]["discount"];
		auto observation = response["time_step"]["observation"];
		auto info = response["time_step"]["info"];

		std::unordered_map<std::string, std::any> info_;
		info_["prob"] = std::any(static_cast<real_t>(info["prob"]));


		return CliffWorld::time_step_type(TimeStepEnumUtils::time_step_type_from_int(step_type),
		                                  reward, observation, discount,
		                                  std::move(info_));

	}


	CliffWorld::CliffWorld(const RESTApiServerWrapper& api_server)
		:
		ToyTextEnvBase<TimeStep<uint_t>, 37, 4>(api_server, 0, CliffWorld::name),
		max_episode_steps_(200)
	{
		this ->get_api_server().register_if_not(CliffWorld::name,CliffWorld::URI);
	}

	CliffWorld::CliffWorld(const RESTApiServerWrapper& api_server,
	                       const uint_t cidx)
		:
		ToyTextEnvBase<TimeStep<uint_t>, 37, 4>(api_server, cidx, CliffWorld::name),
		max_episode_steps_(200)
	{
		this ->get_api_server().register_if_not(CliffWorld::name,CliffWorld::URI);
	}

	CliffWorld::CliffWorld(const CliffWorld& other)
		:
		ToyTextEnvBase<TimeStep<uint_t>, 37, 4>(other),
		max_episode_steps_(other.max_episode_steps_)
	{}

	void
	CliffWorld::make(const std::string& version,
	                 const std::unordered_map<std::string, std::any>& options){

		if(this->is_created()){
			return;
		}

		auto max_episode_steps_itr = options.find("max_episode_steps");
		if( max_episode_steps_itr != options.end()){
			max_episode_steps_ = std::any_cast<uint_t>(max_episode_steps_itr->second);
		}

		nlohmann::json ops;
		ops["max_episode_steps"] = max_episode_steps_;
		auto response = this -> get_api_server().make(this->env_name(),
		                                              this->cidx(),
		                                              version,
		                                              ops);

		this->set_version_(version);
		this->make_created_();

	}

	CliffWorld::time_step_type
	CliffWorld::step(const action_type& action){

#ifdef BITRL_DEBUG
		assert(this->is_created() && "Environment has not been created");
#endif

		if(this->get_current_time_step_().last()){
			return this->reset(42, std::unordered_map<std::string, std::any>());
		}

		auto response = this -> get_api_server().step(this->env_name(),
		                                              this->cidx(),
		                                              action);

		this->get_current_time_step_() = this->create_time_step_from_response_(response);
		return this->get_current_time_step_();

	}

	CliffWorld
	CliffWorld::make_copy(uint_t cidx)const{

		CliffWorld copy(this -> get_api_server(),cidx);
		std::unordered_map<std::string, std::any> ops;
		auto version = this -> version();
		copy.make(version, ops);
		return copy;
	}

}
}
