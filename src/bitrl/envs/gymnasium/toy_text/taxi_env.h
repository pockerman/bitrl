#ifndef TAXI_H
#define TAXI_H

#include "bitrl/bitrl_config.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/envs/gymnasium/toy_text/toy_text_base.h"
#include "bitrl/envs/time_step.h"
#include "bitrl/extern/nlohmann/json/json.hpp"
#include "bitrl/network/rest_rl_env_client.h"

#include <any>
#include <string>
#include <unordered_map>

namespace bitrl
{
namespace envs::gymnasium
{

///
/// \brief The Taxi class
///
class Taxi final : public ToyTextEnvBase<TimeStep<uint_t>, 500, 6>
{
  public:
    ///
    /// \brief name
    ///
    static const std::string name;

    ///
    /// \brief The URI for accessing the environment
    ///
    static const std::string URI;

    ///
    /// \brief The base type
    ///
    typedef typename ToyTextEnvBase<TimeStep<uint_t>, 500, 6>::base_type base_type;

    ///
    /// \brief The time step type we return every time a step in the
    /// environment is performed
    ///
    typedef typename base_type::time_step_type time_step_type;

    ///
    /// \brief The type describing the state space for the environment
    ///
    typedef typename base_type::state_space_type state_space_type;

    ///
    /// \brief The type of the action space for the environment
    ///
    typedef typename base_type::action_space_type action_space_type;

    ///
    /// \brief The type of the action to be undertaken in the environment
    ///
    typedef typename base_type::action_type action_type;

    ///
    /// \brief The type of the action to be undertaken in the environment
    ///
    typedef typename base_type::state_type state_type;

    ///
    /// \brief Taxi
    ///
    Taxi(network::RESTRLEnvClient &api_server);

    ///
    /// \brief copy constructor
    ///
    Taxi(const Taxi &other);

    ///
    /// \brief ~FrozenLake. Destructor.
    ///
    ~Taxi() override = default;

    ///
    /// \brief make. Builds the environment. Optionally we can choose if the
    /// environment will be slippery
    ///
    virtual void
    make(const std::string &version, const std::unordered_map<std::string, std::any> & /*options*/,
         const std::unordered_map<std::string, std::any> & /*reset_options*/) override final;

    ///
    /// \brief step
    ///
    virtual time_step_type step(const action_type &action) override final;

  protected:
    ///
    /// \brief Handle the reset response from the environment server
    ///
    virtual time_step_type
    create_time_step_from_response_(const nlohmann::json &response) const override final;
};

} // namespace envs::gymnasium
} // namespace bitrl
#endif // TAXI_H
