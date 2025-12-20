#include "bitrl/envs/gymnasium/toy_text/frozen_lake_env.h"
#include "bitrl/bitrl_config.h"
#include "bitrl/envs/time_step_type.h"
#include "bitrl/extern/nlohmann/json/json.hpp"


#ifdef BITRL_DEBUG
#include <cassert>
#endif

#include <iostream>
#include <memory>
#include <stdexcept>

namespace bitrl{
namespace envs::gymnasium
{

template<uint_t side_size>
const std::string FrozenLake<side_size>::name = "FrozenLake";

template<uint_t side_size>
const std::string FrozenLake<side_size>::URI = "/gymnasium/frozen-lake-env";

template<uint_t side_size>
FrozenLake<side_size>::FrozenLake(network::RESTRLEnvClient& api_server)
		:
		ToyTextEnvBase<TimeStep<uint_t>,
		               frozenlake_state_size<side_size>::size,
		               3>(api_server, FrozenLake<side_size>::name)
{
	this -> get_api_server().register_if_not(FrozenLake<side_size>::name, FrozenLake<side_size>::URI);
}

template<uint_t side_size>
FrozenLake<side_size>::FrozenLake(const FrozenLake<side_size>& other)
		:
		ToyTextEnvBase<TimeStep<uint_t>,
		               frozenlake_state_size<side_size>::size,
		               3>(other)
	{}

template<uint_t side_size>
typename FrozenLake<side_size>::time_step_type
FrozenLake<side_size>::create_time_step_from_response_(const nlohmann::json& response)const{

		auto step_type = response["time_step"]["step_type"].template get<uint_t>();
		auto step_type_val = TimeStepEnumUtils::time_step_type_from_int(step_type);
		auto reward = response["time_step"]["reward"];
		auto discount = response["time_step"]["discount"];
		auto observation = response["time_step"]["observation"];
		auto info = response["time_step"]["info"];

		auto time_step = FrozenLake<side_size>::time_step_type(step_type_val,
		                                                       reward, observation, discount,
		                                                       std::unordered_map<std::string, std::any>());

		return time_step;
	}

template<uint_t side_size>
bool
FrozenLake<side_size>::is_slippery()const
{
	auto make_ops = this -> make_options();
	auto slip_itr = make_ops.find("is_slippery");

	if( slip_itr != make_ops.end())
	{
		return std::any_cast<bool>(slip_itr->second);
	}

	throw std::logic_error("Property: is_slippery was not found. Have your called make?");
}

template<uint_t side_size>
void
FrozenLake<side_size>::make(const std::string& version,
	                            const std::unordered_map<std::string, std::any>& options,
	                            const std::unordered_map<std::string, std::any>& reset_options){

		if(this->is_created()){
			return;
		}

		std::unordered_map<std::string, std::any> copy_make_ops = options;
		auto is_slippery = true;

		auto slip_itr = options.find("is_slippery");
		if( slip_itr != options.end()){
			is_slippery = std::any_cast<bool>(slip_itr->second);
			copy_make_ops["is_slippery"] = is_slippery;
		}
		else
		{
			copy_make_ops["is_slippery"] = true;
		}


		nlohmann::json ops;
		ops["map_name"] = map_type();
		ops["is_slippery"] = is_slippery;
		auto response = this -> get_api_server().make(this -> env_name(),
		                                              version, ops);

		auto idx = response["idx"];
		this -> set_idx_(idx);
		this -> base_type::make(version, copy_make_ops, reset_options);
		this -> make_created_();
	}

template<uint_t side_size>
typename FrozenLake<side_size>::time_step_type
FrozenLake<side_size>::step(const action_type& action){

#ifdef BITRL_DEBUG
		assert(this->is_created() && "Environment has not been created");
#endif

	if(this->get_current_time_step_().last()){
		return this->reset();
	}

	auto response = this -> get_api_server().step(this -> env_name(), this -> idx(), action);
	this->get_current_time_step_() = this->create_time_step_from_response_(response);
	return this->get_current_time_step_();

}

template class FrozenLake<4>;
template class FrozenLake<8>;

}
}
