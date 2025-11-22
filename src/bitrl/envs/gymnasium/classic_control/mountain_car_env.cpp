#include "bitrl/bitrl_config.h"
#include "bitrl/envs/gymnasium/classic_control/mountain_car_env.h"
#include "bitrl/envs/time_step.h"
#include "bitrl/envs/time_step_type.h"
#include "bitrl/extern/nlohmann/json/json.hpp"


#ifdef BITRL_DEBUG
#include <cassert>
#endif

#include <string>
#include <vector>
#include <tuple>
#include <any>

namespace bitrl{
namespace envs::gymnasium
{


	const std::string MountainCar::name = "MountainCar";
	const std::string MountainCar::URI =  "/gymnasium/mountain-car-env";

	MountainCar::time_step_type
	MountainCar::create_time_step_from_response_(const nlohmann::json& response)const{

		auto step_type = response["time_step"]["step_type"].template get<uint_t>();
		auto reward    = response["time_step"]["reward"];
		auto discount  = response["time_step"]["discount"];
		auto obs       = response["time_step"]["observation"];
		auto info      = response["time_step"]["info"];
		return MountainCar::time_step_type(TimeStepEnumUtils::time_step_type_from_int(step_type),
		                                   reward, obs, discount,
		                                   std::unordered_map<std::string, std::any>());
	}


	MountainCar::MountainCar(const RESTApiServerWrapper& api_server )
		:
		GymnasiumEnvBase<TimeStep<std::vector<real_t>>,
		                 ContinuousVectorStateDiscreteActionEnv<3, 2, 0, real_t >
		>(api_server, 0, MountainCar::name)
	{
		this -> get_api_server().register_if_not(MountainCar::name, MountainCar::URI);
	}


	MountainCar::MountainCar(const RESTApiServerWrapper& api_server ,
	                         const uint_t cidx)
		:
		GymnasiumEnvBase<TimeStep<std::vector<real_t>>,
		                 ContinuousVectorStateDiscreteActionEnv<3, 2, 0, real_t >
		>(api_server, cidx, MountainCar::name)
	{
		this -> get_api_server().register_if_not(MountainCar::name, MountainCar::URI);
	}

	MountainCar::MountainCar(const MountainCar& other)
		:
		GymnasiumEnvBase<TimeStep<std::vector<real_t>>,
		                 ContinuousVectorStateDiscreteActionEnv<3, 2, 0, real_t >
		>(other)
	{}

	void
	MountainCar::make(const std::string& version,
	                  const std::unordered_map<std::string, std::any>& /*options*/){

		if(this->is_created()){
			return;
		}

		auto response  = this -> get_api_server().make(this -> env_name(),
		                                               this -> cidx(),
		                                               version, nlohmann::json());

		this->set_version_(version);
		this->make_created_();

		this->set_version_(version);
		this->make_created_();

	}

	MountainCar::time_step_type
	MountainCar::step(const action_type& action){

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

	MountainCar
	MountainCar::make_copy(uint_t cidx)const{

		MountainCar copy(this -> get_api_server(), cidx);
		std::unordered_map<std::string, std::any> ops;
		auto version = this -> version();
		copy.make(version, ops);
		return copy;
	}

}
}
