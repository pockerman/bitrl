//
// Created by alex on 6/29/25.
//

#include "bitrl/envs/gymnasium/box2d/lunar_lander_env.h"

namespace bitrl
{
namespace envs::gymnasium
{

const std::string LunarLanderDiscreteEnv::name = "LunarLanderDiscrete";
const std::string LunarLanderDiscreteEnv::URI = "/gymnasium/lunar-lander-discrete-env";

const std::string LunarLanderContinuousEnv::name = "LunarLanderContinuous";
const std::string LunarLanderContinuousEnv::URI = "/gymnasium/lunar-lander-continuous-env";

LunarLanderDiscreteEnv::LunarLanderDiscreteEnv(network::RESTRLEnvClient &api_server)
    : LunarLanderDiscreteEnv::base_type(api_server, LunarLanderDiscreteEnv::name,
                                        LunarLanderDiscreteEnv::URI)
{
}

LunarLanderDiscreteEnv::LunarLanderDiscreteEnv(const LunarLanderDiscreteEnv &other)
    : LunarLanderDiscreteEnv::base_type(other)
{
}

LunarLanderContinuousEnv::LunarLanderContinuousEnv(network::RESTRLEnvClient &api_server)
    : LunarLanderContinuousEnv::base_type(api_server, LunarLanderContinuousEnv::name,
                                          LunarLanderContinuousEnv::URI)
{
}

LunarLanderContinuousEnv::LunarLanderContinuousEnv(const LunarLanderContinuousEnv &other)
    : LunarLanderContinuousEnv::base_type(other)
{
}

} // namespace envs::gymnasium
} // namespace bitrl
