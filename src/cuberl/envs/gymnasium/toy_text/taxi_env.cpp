#include "cuberl/envs/gymnasium/toy_text/taxi_env.h"
#include "cuberl/base/cuberl_config.h"
#include "cuberl/envs/time_step_type.h"
#include "bitrl/extern/nlohmann/json/json.hpp"

#ifdef CUBERL_DEBUG
#include <cassert>
#include <iostream>
#endif

namespace cuberl
{
namespace envs::gymnasium
{

const std::string Taxi::name = "Taxi";
const std::string Taxi::URI = "/gymnasium/taxi-env";

Taxi::Taxi(network::RESTRLEnvClient &api_server)
    : ToyTextEnvBase<TimeStep<uint_t>, 500, 6>(api_server, Taxi::name)
{

    this->get_api_server().register_if_not(Taxi::name, Taxi::URI);
}

Taxi::time_step_type Taxi::create_time_step_from_response_(const nlohmann::json &response) const
{

    auto step_type = response["time_step"]["step_type"].template get<uint_t>();
    auto reward = response["time_step"]["reward"];
    auto discount = response["time_step"]["discount"];
    auto observation = response["time_step"]["observation"];
    auto info = response["time_step"]["info"];
    return Taxi::time_step_type(TimeStepEnumUtils::time_step_type_from_int(step_type), reward,
                                observation, discount, std::unordered_map<std::string, std::any>());
}

void Taxi::make(const std::string &version,
                const std::unordered_map<std::string, std::any> &options,
                const std::unordered_map<std::string, std::any> &reset_options)
{

    if (this->is_created())
    {
        return;
    }

    auto response = this->get_api_server().make(this->env_name(), version, nlohmann::json());

    auto idx = response["idx"];
    this->set_idx_(idx);
    this->base_type::make(version, options, reset_options);
    this->make_created_();
}

Taxi::time_step_type Taxi::step(const action_type &action)
{

#ifdef CUBERL_DEBUG
    assert(this->is_created() && "Environment has not been created");
#endif

    if (this->get_current_time_step_().last())
    {
        return this->reset();
    }

    auto response = this->get_api_server().step(this->env_name(), this->idx(), action);

    this->get_current_time_step_() = this->create_time_step_from_response_(response);
    return this->get_current_time_step_();
}

} // namespace envs::gymnasium
} // namespace cuberl
