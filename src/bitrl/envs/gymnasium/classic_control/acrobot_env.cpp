#include "bitrl/envs/gymnasium/classic_control/acrobot_env.h"
#include "bitrl/bitrl_types_v2.h"
#include "bitrl/envs/time_step.h"
#include "bitrl/envs/time_step_type.h"
#include "bitrl/rlenvscpp_config.h"
#include "bitrl/extern/nlohmann/json/json.hpp"

namespace bitrl{
namespace envs{
namespace gymnasium{
	
const std::string Acrobot::name = "Acrobot";
const std::string Acrobot::URI = "/gymnasium/acrobot-env";

Acrobot::time_step_type
Acrobot::create_time_step_from_response_(const nlohmann::json& response)const{

    auto step_type = response["time_step"]["step_type"].template get<uint_t>();
    auto reward    = response["time_step"]["reward"];
    auto discount  = response["time_step"]["discount"];
    auto obs       = response["time_step"]["observation"];
    auto info      = response["time_step"]["info"];
    return Acrobot::time_step_type(TimeStepEnumUtils::time_step_type_from_int(step_type),
									reward, obs, discount,
									std::unordered_map<std::string, std::any>());
}


Acrobot::Acrobot(const RESTApiServerWrapper& api_server)
:
GymnasiumEnvBase<TimeStep<std::vector<real_t> >,
				 ContinuousVectorStateDiscreteActionEnv<6, 2, 0, real_t > 
				>(api_server, 
				  0, 
				  Acrobot::name)
{
	this -> get_api_server().register_if_not(Acrobot::name, Acrobot::URI);
}

Acrobot::Acrobot(const RESTApiServerWrapper& api_server, 
		         const uint_t cidx)
:
GymnasiumEnvBase<TimeStep<std::vector<real_t> >,
				 ContinuousVectorStateDiscreteActionEnv<6, 2, 0, real_t > 
				>(api_server, 
				  cidx, 
				  Acrobot::name)
{
	this -> get_api_server().register_if_not(Acrobot::name, Acrobot::URI);
}

Acrobot::Acrobot(const Acrobot& other)
:
GymnasiumEnvBase<TimeStep<std::vector<real_t> >,
				 ContinuousVectorStateDiscreteActionEnv<6, 2, 0, real_t > 
				>(other)
{}

void
Acrobot::make(const std::string& version,
              const std::unordered_map<std::string, std::any>& /*options*/){

    if(this->is_created()){
        return;
    }

	auto response  = this -> get_api_server().make(this -> env_name(),
												   this -> cidx(),
												   version, nlohmann::json());

    this->set_version_(version);
    this->make_created_();
}


Acrobot::time_step_type
Acrobot::step(const action_type& action){

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

Acrobot 
Acrobot::make_copy(uint_t cidx)const{
	
	Acrobot copy(this->get_api_server(), cidx);
	
	std::unordered_map<std::string, std::any> ops;
	auto version = this -> version();
	copy.make(version, ops);
	return copy;
}

	
	
}
}
}