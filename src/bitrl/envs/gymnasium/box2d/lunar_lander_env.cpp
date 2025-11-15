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

        LunarLanderDiscreteEnv::LunarLanderDiscreteEnv(const RESTApiServerWrapper& api_server)
        :
        LunarLanderDiscreteEnv::base_type(api_server, 0, LunarLanderDiscreteEnv::name, LunarLanderDiscreteEnv::URI)
        {}

        LunarLanderDiscreteEnv::LunarLanderDiscreteEnv(const RESTApiServerWrapper& api_server, const uint_t cidx)
        :
        LunarLanderDiscreteEnv::base_type(api_server, cidx, LunarLanderDiscreteEnv::name, LunarLanderDiscreteEnv::URI)
        {}

        LunarLanderDiscreteEnv::LunarLanderDiscreteEnv(const LunarLanderDiscreteEnv& other)
            :
        LunarLanderDiscreteEnv::base_type(other)
        {}

        LunarLanderDiscreteEnv
        LunarLanderDiscreteEnv::make_copy(uint_t cidx)const
        {

            LunarLanderDiscreteEnv copy(this -> get_api_server(), cidx);
            auto version = this -> version();
            copy.make(version, this -> make_options());
            return copy;

        }

        LunarLanderContinuousEnv::LunarLanderContinuousEnv(const RESTApiServerWrapper& api_server)
        :
        LunarLanderContinuousEnv::base_type(api_server, 0, LunarLanderContinuousEnv::name, LunarLanderContinuousEnv::URI)
        {}

        LunarLanderContinuousEnv::LunarLanderContinuousEnv(const RESTApiServerWrapper& api_server, const uint_t cidx)
        :
        LunarLanderContinuousEnv::base_type(api_server, cidx, LunarLanderContinuousEnv::name, LunarLanderContinuousEnv::URI)
        {}

        LunarLanderContinuousEnv::LunarLanderContinuousEnv(const LunarLanderContinuousEnv& other)
            :
        LunarLanderContinuousEnv::base_type(other)
        {}

        LunarLanderContinuousEnv
        LunarLanderContinuousEnv::make_copy(uint_t cidx)const
        {

            LunarLanderContinuousEnv copy(this -> get_api_server(), cidx);
            auto version = this -> version();
            copy.make(version, this -> make_options());
            return copy;

        }

    }
}
