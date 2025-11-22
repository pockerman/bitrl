//
// Created by alex on 11/22/25.
//

#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/rigid_bodies/chrono_quadrotor.h"
#include <chrono/physics/ChBodyEasy.h>

namespace bitrl
{
    namespace rigid_bodies
    {

        ChQuadrotor::ChQuadrotor(const ChQuadrotorConfig& config)
            :
        bitrl::dynamics::MotionModelDynamicsBase<bitrl::dynamics::SysState<12>, Null, RealVec>(),
        config_(config),
        quad_(nullptr),
        system_()
        {}


        ChQuadrotor::ChQuadrotor(const ChQuadrotorConfig& config, const state_type& state)
            :
        bitrl::dynamics::MotionModelDynamicsBase<bitrl::dynamics::SysState<12>, Null, RealVec>(state),
        config_(config),
        quad_(nullptr),
        system_()
        {
            init(state);
        }

        void ChQuadrotor::init(const state_type& state)
        {

            quad_ = std::make_shared<chrono::ChBodyEasyBox>(config_.lx, config_.ly, config_.lz, config_.rho_,             // density
                                                false, false);

            quad_ -> SetMass(config_.mass);

            auto x = state["x"];
            auto y = state["y"];
            auto z = state["z"];

            quad_ -> SetPos({x, y, z});


            // add the quad to the system
            system_.Add(quad_);

        }

    }
}


#endif
