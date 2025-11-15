#include "bitrl/envs/gymnasium/classic_control/vector/acrobot_vec_env.h"
#include "bitrl/envs/vector_time_step.h"
#include "bitrl/envs/time_step_type.h"
#include "bitrl/extern/nlohmann/json/json.hpp"

#include <iostream>

namespace bitrl{
namespace envs{
namespace gymnasium{
	

const std::string AcrobotV::name = "AcrobotV";
const std::string AcrobotV::URI = "/gymnasium/acrobot-env/v";


AcrobotV::time_step_type
AcrobotV::create_time_step_from_response_(const nlohmann::json& response)const{

    auto step_types        = response["time_step"]["step_types"].template get<std::vector<uint_t> >(); 
	auto time_step_types   = TimeStepEnumUtils::time_step_type_from_int(step_types);
    auto reward_response   = response["time_step"]["rewards"];
    auto discount_response = response["time_step"]["discounts"];
    auto observation       = response["time_step"]["observations"];
    auto info              = response["time_step"]["infos"];
    return AcrobotV::time_step_type(time_step_types,
									reward_response, 
									observation, 
									discount_response,
									std::unordered_map<std::string, std::any>());
}
	
	
AcrobotV::AcrobotV(const RESTApiServerWrapper& api_server )
:
GymnasiumVecEnvBase<VectorTimeStep<detail_::AcrobotVEnv::state_type>, 
					detail_::AcrobotVEnv>(api_server, 0, AcrobotV::name)
{
	this -> get_api_server().register_if_not(AcrobotV::name,AcrobotV::URI);
}

AcrobotV::AcrobotV(const RESTApiServerWrapper& api_server , const uint_t cidx)
:
GymnasiumVecEnvBase<VectorTimeStep<detail_::AcrobotVEnv::state_type>, 
                    detail_::AcrobotVEnv>(api_server, cidx, 
										  AcrobotV::name)
{
	this -> get_api_server().register_if_not(AcrobotV::name,AcrobotV::URI);
}


AcrobotV::AcrobotV(const AcrobotV& other)
:
GymnasiumVecEnvBase<VectorTimeStep<detail_::AcrobotVEnv::state_type>, 
                    detail_::AcrobotVEnv>(other)
{}


void
AcrobotV::make(const std::string& version,
              const std::unordered_map<std::string, std::any>& options){

    if(this->is_created()){
        return;
    }
	
	this->GymnasiumVecEnvBase<VectorTimeStep<detail_::AcrobotVEnv::state_type>,
                             detail_::AcrobotVEnv>::make(version, options);
			
	nlohmann::json ops;
	ops["num_envs"] = this->get_n_envs();
	auto response = this -> get_api_server().make(this -> env_name(),
	                                              this -> cidx(),
												  version, ops);

    this->set_version_(version);
    this->make_created_();
}


AcrobotV::time_step_type
AcrobotV::step(const action_type& action){

	
#ifdef BITRL_DEBUG
     assert(this->is_created() && "Environment has not been created");
#endif

     if(this->get_reset_if_any_done() && this->get_current_time_step_().last()){
         return this->reset(42, std::unordered_map<std::string, std::any>());
     }
	 
	 auto response = this -> get_api_server().step(this -> env_name(),
	                                              this -> cidx(),
												  action);

    this->get_current_time_step_() = this->create_time_step_from_response_(response);
    return this->get_current_time_step_();
}


AcrobotV
AcrobotV::make_copy(uint_t cidx)const{
	
	AcrobotV copy(this->get_api_server(), cidx);
	
	std::unordered_map<std::string, std::any> ops;
	ops["num_envs"] = this -> get_n_envs();
	auto version = this -> version();
	copy.make(version, ops);
	return copy;
}
	
}
}
}