//
// Created by alex on 11/22/25.
//

#ifndef CHRONO_QUADROTOR_H
#define CHRONO_QUADROTOR_H

#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/bitrl_types.h"
#include "bitrl/dynamics/motion_model_base.h"
#include "bitrl/dynamics/system_state.h"


#include <chrono/physics/ChSystemNSC.h>
#include <chrono/physics/ChBodyEasy.h>

#include <memory>

namespace bitrl
{
    namespace rigid_bodies
    {
        struct ChQuadrotorConfig
        {
            ///
            /// We model the quadrotor as a box.
            /// These are the assumed lengths in x, y, z directions
            ///
            real_t lx {0.5};
            real_t ly {0.5};
            real_t lz {0.5};
            ///
            /// Mass of the quadrotor
            ///
            real_t mass;

            ///
            /// Arm length of the quadrotor
            ///
            real_t l;

            ///
            /// The density of the quadrotor
            ///
            real_t rho_{1.0};

            ///
            /// Inertial matrix
            ///
            SquareMat<real_t, 3> J;

        };

        class ChQuadrotor: public bitrl::dynamics::MotionModelDynamicsBase<bitrl::dynamics::SysState<12>, Null, RealVec>
        {
        public:

            typedef bitrl::dynamics::SysState<12> state_type;


            ///  Constructor
            /// @param config
            ///
            ChQuadrotor(const ChQuadrotorConfig& config);

            ///
            /// @param config
            /// @param state
            ChQuadrotor(const ChQuadrotorConfig& config, const state_type& state);

            ///  Builds the underlying ChSystemNSC
            ///  and initialises the state of the Quadrotor
            /// @param state
            void init(const state_type& state);

        private:

            ///
            /// The configuration of the quadrotor
            ChQuadrotorConfig config_;

            std::shared_ptr<chrono::ChBodyEasyBox> quad_;
            ///
            /// The underlying Chrono system
            ///
            chrono::ChSystemNSC system_;

        };

    }
}

#endif
#endif //CHRONO_QUADROTOR_H
