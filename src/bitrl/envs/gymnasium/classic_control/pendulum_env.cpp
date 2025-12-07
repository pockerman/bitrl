#include "bitrl/envs/gymnasium/classic_control/pendulum_env.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/envs/time_step.h"
#include "bitrl/envs/time_step_type.h"
#include "bitrl/bitrl_config.h"
#include "bitrl/extern/nlohmann/json/json.hpp"


namespace bitrl{
namespace envs{
namespace gymnasium{
	
	// static data
const std::string Pendulum::name = "Pendulum";
const std::string Pendulum::URI = "/gymnasium/pendulum-env";

Pendulum::time_step_type
Pendulum::create_time_step_from_response_(const nlohmann::json& response)const{

    auto step_type = response["time_step"]["step_type"].template get<uint_t>();
    auto reward    = response["time_step"]["reward"];
    auto discount  = response["time_step"]["discount"];
    auto obs       = response["time_step"]["observation"];
    auto info      = response["time_step"]["info"];
    return Pendulum::time_step_type(TimeStepEnumUtils::time_step_type_from_int(step_type),
									reward, obs, discount,
									std::unordered_map<std::string, std::any>());
}


Pendulum::Pendulum(const RESTRLEnvClient& api_server)
:
GymnasiumEnvBase<TimeStep<std::vector<real_t>>, 
				 ContinuousVectorStateContinuousScalarBoundedActionEnv<3, 
																	   1, 
											                           RealRange<-2.0, 2.0>, 
																	   0, real_t>
											 >(api_server, 0, Pendulum::name)
{
	this -> get_api_server().register_if_not(Pendulum::name,Pendulum::URI);
}

Pendulum::Pendulum(const RESTRLEnvClient& api_server, 
	               const uint_t cidx)
:
GymnasiumEnvBase<TimeStep<std::vector<real_t>>, 
				 ContinuousVectorStateContinuousScalarBoundedActionEnv<3, 
																	   1, 
											                           RealRange<-2.0, 2.0>, 
																	   0, real_t>
											 >(api_server,
											   cidx, 
											   Pendulum::name)
{
 this -> get_api_server().register_if_not(Pendulum::name,Pendulum::URI);	
}

Pendulum::Pendulum(const Pendulum& other)
:
GymnasiumEnvBase<TimeStep<std::vector<real_t>>, 
				 ContinuousVectorStateContinuousScalarBoundedActionEnv<3, 
																	   1, 
											                           RealRange<-2.0, 2.0>, 
																	   0, real_t>
											 >(other)
{}

void
Pendulum::make(const std::string& version,
              const std::unordered_map<std::string, std::any>& /*options*/){

    if(this->is_created()){
        return;
    }
	
	auto response = this -> get_api_server().make(this -> env_name(),
	                                              this -> cidx(),
												  version, nlohmann::json());

    this->set_version_(version);
    this->make_created_();
}


Pendulum::time_step_type
Pendulum::step(const action_type& action){

#ifdef BITRL_DEBUG
     assert(this->is_created() && "Environment has not been created");
#endif

     if(this->get_current_time_step_().last()){
         return this->reset(42, std::unordered_map<std::string, std::any>());
     }

	auto response = this -> get_api_server().step(this -> env_name(),
	                                              this -> cidx(),
												  action);

    this->get_current_time_step_() = this->create_time_step_from_response_(response);
    return this->get_current_time_step_();
}


Pendulum 
Pendulum::make_copy(uint_t cidx)const{
	Pendulum copy(this -> get_api_server(), cidx);
	std::unordered_map<std::string, std::any> ops;
	auto version = this -> version();
	copy.make(version, ops);
	return copy;
}

}
}
}