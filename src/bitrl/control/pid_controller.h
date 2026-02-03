#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "bitrl/bitrl_types.h"

#include <utility>

namespace bitrl
{
    namespace control
    {
    /**
 * @class PIDController
 * @ingroup bitrl_control
 * @brief PID controller implementation.
 *
 */
        class PIDController
        {
            public:

            typedef real_t ctrl_signal_type;

            /// Constructor
            /// @param Kp
            /// @param Ki
            /// @param Kd
            PIDController(real_t Kp, real_t Ki, real_t Kd);

            real_t get_kp()const noexcept{return kp_;}
            real_t get_ki()const noexcept{return ki_;}
            real_t get_kd()const noexcept{return kd_;}

            /// Test the is_limited_ flag
            /// @return True is the set_control_limits has been called
            bool is_limited()const noexcept{return is_limited_;}

            /// Set the min/max threshold for the action
            /// @param limits
            void set_control_limits(std::pair<real_t, real_t> limits);

            /// Compute the control signal to return
            /// @param current
            /// @param target
            /// @param dt
            /// @return
            ctrl_signal_type compute_ctrl_signal(real_t current, real_t target, real_t dt);

        private:

            bool is_limited_{false};
            real_t kp_;
            real_t ki_;
            real_t kd_;

            real_t integral_{0.0};
            real_t err_old_{0.0};
            // min = limits.first, max = limits.second
            std::pair<real_t, real_t> limits_;

        };

        inline
        PIDController::PIDController(real_t kp, real_t ki, real_t kd)
            :
        kp_(kp),
        ki_(ki),
        kd_(kd)
        {}

        inline
        void
        PIDController::set_control_limits(std::pair<real_t, real_t> limits)
        {
            is_limited_ = true;
            limits_ = limits;
        }
    }

}

#endif //PID_CONTROLLER_H
