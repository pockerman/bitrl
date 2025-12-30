/*
 * GymWalk environment from
 * <a href="https://github.com/mimoralea/gym-walk2">gym_walk</a>
 *
 *
 *
 */

#ifndef GYM_WALK_H
#define GYM_WALK_H

#include "bitrl/bitrl_config.h"
#include "bitrl/bitrl_consts.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/envs/env_base.h"
#include "bitrl/envs/env_types.h"
#include "bitrl/envs/time_step.h"
#include "bitrl/network/rest_rl_env_client.h"
#include "bitrl/utils//std_map_utils.h"
#include "bitrl/extern/nlohmann/json/json.hpp"

#include <any>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#ifdef BITRL_DEBUG
#include <cassert>
#endif

namespace bitrl
{
namespace envs::gdrl
{

/**
 * GymWalk. Interface for the GymWalk environment
 */
template <uint_t state_size>
class GymWalk final : public EnvBase<TimeStep<uint_t>, ScalarDiscreteEnv<state_size, 1, 0, 0>>
{
  public:

    static const std::string name;
    static const std::string URI;

    typedef EnvBase<TimeStep<uint_t>, ScalarDiscreteEnv<state_size, 1, 0, 0>> base_type;
    typedef typename base_type::time_step_type time_step_type;
    typedef typename base_type::state_space_type state_space_type;
    typedef typename base_type::action_space_type action_space_type;
    typedef typename base_type::action_type action_type;
    typedef typename base_type::state_type state_type;
    typedef std::vector<std::tuple<real_t, uint_t, real_t, bool>> dynamics_t;

    ///
    /// \brief Constructor
    ///
    GymWalk(network::RESTRLEnvClient &api_server);

    virtual void
    make(const std::string &version, const std::unordered_map<std::string, std::any> &options,
         const std::unordered_map<std::string, std::any> &reset_options) override final;
    bool is_alive() const;
    virtual void close() override final;
    virtual time_step_type step(const action_type &action) override final;
    virtual time_step_type reset() override final;
    uint_t n_states() const noexcept { return state_space_type::size; }
    uint_t n_actions() const noexcept { return action_space_type::size + 1; }
    dynamics_t p(uint_t sidx, uint_t aidx) const;

    /**
     * @brief Get the full URL for this environment endpoint on the server.
     * @return Environment URL string.
     */
    std::string get_url() const{return api_server_->get_env_url(this->env_name());}

    /**
    * Get the number of copies on the server for this environment
    * @return
    */
    uint_t n_copies() const;

  private:
    dynamics_t build_dynamics_from_response_(const nlohmann::json &response) const;
    time_step_type create_time_step_from_response_(const nlohmann::json &response) const;

    network::RESTRLEnvClient *api_server_;
};

template <uint_t state_size> const std::string GymWalk<state_size>::name = "GymWalk";
template <uint_t state_size> const std::string GymWalk<state_size>::URI = "/gdrl/gym-walk-env";

template <uint_t state_size>
GymWalk<state_size>::GymWalk(network::RESTRLEnvClient &api_server)
    : base_type(bitrl::consts::INVALID_STR, name),
      api_server_(&api_server)
{
    api_server_ -> register_if_not(name, URI);
}

template <uint_t state_size>
typename GymWalk<state_size>::dynamics_t
GymWalk<state_size>::build_dynamics_from_response_(const nlohmann::json &response) const
{
    auto dynamics = response["dynamics"];
    return dynamics;
}

template <uint_t state_size>
typename GymWalk<state_size>::time_step_type
GymWalk<state_size>::create_time_step_from_response_(const nlohmann::json &response) const
{

    auto step_type = response["time_step"]["step_type"];
    auto reward = response["time_step"]["reward"];
    auto discount = response["time_step"]["discount"];
    auto observation = response["time_step"]["observation"];
    auto info = response["time_step"]["info"];
    return GymWalk::time_step_type(TimeStepEnumUtils::time_step_type_from_int(step_type.get<uint_t>()), reward, observation,
                                   discount, std::unordered_map<std::string, std::any>());
}

template <uint_t state_size>
void GymWalk<state_size>::make(const std::string &version,
                               const std::unordered_map<std::string, std::any> &options,
                               const std::unordered_map<std::string, std::any> &reset_options)
{

    if (this->is_created())
    {
        return;
    }

    nlohmann::json ops;
    auto response = api_server_ -> make(this->env_name(), version, ops);

    auto idx = response["idx"];
    this->set_idx_(idx);
    this->base_type::make(version, options, reset_options);
    this->make_created_();
}

template <uint_t state_size>
typename GymWalk<state_size>::time_step_type GymWalk<state_size>::step(const action_type &action)
{

#ifdef BITRL_DEBUG
    assert(this->is_created() && "Environment has not been created");
#endif

    if (this->get_current_time_step_().last())
    {
        return this->reset();
    }

    const auto response = api_server_ -> step(this->env_name(), this->idx(), action);

    this->get_current_time_step_() = this->create_time_step_from_response_(response);
    return this->get_current_time_step_();
}

template <uint_t state_size> bool GymWalk<state_size>::is_alive() const
{
    auto response = this->api_server_ -> is_alive(this->env_name(), this->idx());
    return response["result"];
}

template <uint_t state_size>
uint_t GymWalk<state_size>::n_copies() const
{
    auto response = this->api_server_->n_copies(this->env_name());
    return response["copies"];
}


template <uint_t state_size> void GymWalk<state_size>::close()
{

    if (!this->is_created())
    {
        return;
    }

    auto response = this->api_server_ -> close(this->env_name(), this->idx());
    this->invalidate_is_created_flag_();
}

template <uint_t state_size>
typename GymWalk<state_size>::time_step_type GymWalk<state_size>::reset()
{

    if (!this->is_created())
    {
#ifdef BITRL_DEBUG
        assert(this->is_created() && "Environment has not been created");
#endif
        return time_step_type();
    }


    auto &reset_ops = this->reset_options();
    auto seed = utils::resolve<uint_t>("seed", reset_ops);
    auto response = this->api_server_ -> reset(this->env_name(), this->idx(), seed, nlohmann::json());

    this->create_time_step_from_response_(response);
    return this->get_current_time_step_();
}

template <uint_t state_size>
typename GymWalk<state_size>::dynamics_t
GymWalk<state_size>::p(uint_t sidx, uint_t aidx) const
{

#ifdef BITRL_DEBUG
    assert(this->is_created() && "Environment has not been created");
#endif

    auto response = this->api_server_ -> dynamics(this->env_name(), this->idx(), sidx, aidx);
    return build_dynamics_from_response_(response);
}

} // namespace envs::gdrl
} // namespace bitrl

#endif // GYM_WALK_H
