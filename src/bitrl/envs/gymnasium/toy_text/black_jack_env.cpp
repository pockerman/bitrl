#include "bitrl/envs/gymnasium/toy_text/black_jack_env.h"
#include "bitrl/bitrl_config.h"
#include "bitrl/envs/time_step_type.h"
#include "bitrl/extern/nlohmann/json/json.hpp"

#ifdef BITRL_DEBUG
#include <cassert>
#endif

namespace bitrl
{
namespace envs::gymnasium
{

const std::string BlackJack::name = "BlackJack";
const std::string BlackJack::URI = "/gymnasium/black-jack-env";

BlackJack::dynamics_t
BlackJack::build_dynamics_from_response_(const nlohmann::json & /*response*/) const
{
    throw std::logic_error("Environment BlackJack does not have dynamics");
}

BlackJack::time_step_type
BlackJack::create_time_step_from_response_(const nlohmann::json &response) const
{

    auto step_type = TimeStepEnumUtils::time_step_type_from_int(
        response["time_step"]["step_type"].template get<uint_t>());
    auto reward = response["time_step"]["reward"];
    auto discount = response["time_step"]["discount"];
    auto observation = response["time_step"]["observation"];
    auto info = response["time_step"]["info"];

    std::vector<std::tuple<uint_t, uint_t, uint_t>> state(1, observation);
    return BlackJack::time_step_type(step_type, reward, observation, discount,
                                     std::unordered_map<std::string, std::any>());
}

BlackJack::BlackJack(network::RESTRLEnvClient &api_server)
    : ToyTextEnvBase<TimeStep<std::vector<uint_t>>, 48, 2>(api_server, BlackJack::name)
{
    this->get_api_server().register_if_not(BlackJack::name, BlackJack::URI);
}

BlackJack::BlackJack(const BlackJack &other)
    : ToyTextEnvBase<TimeStep<std::vector<uint_t>>, 48, 2>(other), is_natural_(other.is_natural_),
      is_sab_(other.is_sab_)
{
}

void BlackJack::make(const std::string &version,
                     const std::unordered_map<std::string, std::any> &options,
                     const std::unordered_map<std::string, std::any> &reset_options)
{

    if (this->is_created())
    {
        return;
    }

    auto natural_itr = options.find("natural");
    if (natural_itr != options.end())
    {
        is_natural_ = std::any_cast<bool>(natural_itr->second);
    }

    auto sab_itr = options.find("sab");
    if (sab_itr != options.end())
    {
        is_sab_ = std::any_cast<bool>(sab_itr->second);
    }

    nlohmann::json ops;
    ops["natural"] = is_natural_;
    ops["sab"] = is_sab_;
    auto response = this->get_api_server().make(this->env_name(), version, ops);

    auto idx = response["idx"];
    this->set_idx_(idx);
    this->base_type::make(version, options, reset_options);
    this->make_created_();
}

BlackJack::time_step_type BlackJack::step(const action_type &action)
{

#ifdef BITRL_DEBUG
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
} // namespace bitrl
