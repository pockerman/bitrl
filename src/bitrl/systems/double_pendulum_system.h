//
// Created by alex on 11/15/25.
//

#ifndef DOUBLE_PENDULUM_SYSTEM_H
#define DOUBLE_PENDULUM_SYSTEM_H

#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

/**
 *Chrono includes
 */
#include "bitrl/bitrl_types.h"
#include <chrono/physics/ChSystemNSC.h>
//#include <chrono/physics/ChBodyEasy.h>
//#include <chrono/physics/ChLinkRevolute.h>
//#include <chrono/utils/ChUtilsCreators.h>

namespace bitrl
{
    namespace systems
    {
        struct DoublePendulumProperties
        {
            real_t l1 = 1.0;
            real_t l2 = 1.0;
            real_t r1 = 1.0;
            real_t r2 = 1.0;
            real_t density = 1000.;

        };
        class DoublePendulumSystem
        {
        public:

            DoublePendulumSystem();

            /**
             * Build the system
             * @param props
             */
            void build_system(const DoublePendulumProperties& props);


            /**
             * Enable the gravity vector
             */
            void enable_gravity();

            bool has_gravity_enabled() const noexcept{return gravity_enabled_;}
            bool is_initialized() const noexcept{return is_initialized_;}



        protected:

            chrono::ChSystemNSC system_;

        private:

            bool gravity_enabled_{false};
            bool is_initialized_{false};
        };
    }
}

#endif
#endif //DOUBLE_PENDULUM_SYSTEM_H
