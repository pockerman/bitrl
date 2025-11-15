#include "bitrl/envs/gymnasium/toy_text/taxi_env.h"
#include "bitrl/rlenvscpp_config.h"
#include "bitrl/envs/time_step_type.h"
#include "bitrl/extern/nlohmann/json/json.hpp"


#ifdef BITRL_DEBUG
#include <cassert>
#include <iostream>
#endif

#include <memory>

namespace bitrl{
namespace envs{
namespace gymnasium{

const std::string Taxi::name = "Taxi";
const std::string Taxi::URI = "/gymnasium/taxi-env";

Taxi::Taxi(const RESTApiServerWrapper& api_server)
    :
     ToyTextEnvBase<TimeStep<uint_t>, 500, 6>(api_server, 0, Taxi::name)
{

	this ->get_api_server().register_if_not(Taxi::name,Taxi::URI); 
}

Taxi::Taxi(const RESTApiServerWrapper& api_server, const uint_t cidx)
		   :
ToyTextEnvBase<TimeStep<uint_t>, 500, 6>(api_server, cidx,Taxi::name)
{
	this ->get_api_server().register_if_not(Taxi::name,Taxi::URI); 
}
Taxi::Taxi(const Taxi& other)
:
ToyTextEnvBase<TimeStep<uint_t>, 500, 6>(other)
{}


Taxi::time_step_type
Taxi::create_time_step_from_response_(const nlohmann::json& response)const{

    auto step_type = response["time_step"]["step_type"].template get<uint_t>();
    auto reward = response["time_step"]["reward"];
    auto discount = response["time_step"]["discount"];
    auto observation = response["time_step"]["observation"];
    auto info = response["time_step"]["info"];
    return Taxi::time_step_type(TimeStepEnumUtils::time_step_type_from_int(step_type),
                                reward, observation, discount,
                                std::unordered_map<std::string, std::any>());
}

void
Taxi::make(const std::string& version,
		   const std::unordered_map<std::string, std::any>& /*options*/){

     if(this->is_created()){
        return;
    }
	
	this -> get_api_server().make(this->env_name(),
	                              this->cidx(),
								  version, 
								  nlohmann::json());

	this->set_version_(version);
    this->make_created_();
}


Taxi::time_step_type
Taxi::step(const action_type& action){

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

Taxi
Taxi::make_copy(uint_t cidx)const{
	
	Taxi copy(this -> get_api_server(), cidx);
	std::unordered_map<std::string, std::any> ops;
	auto version = this -> version();
	copy.make(version, ops);
	return copy;
												   
}

}
}
}
