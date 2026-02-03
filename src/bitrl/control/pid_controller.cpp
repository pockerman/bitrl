#include "bitrl/control/pid_controller.h"
#include "bitrl/bitrl_types.h"


namespace bitrl
{
    namespace control
    {
        real_t
        PIDController::compute_ctrl_signal(real_t current, real_t target, real_t dt)
        {
            const auto err = target - current;
            const auto P_term = get_kp() * err;

            integral_ += err * dt;
            const auto I_term = get_ki() * integral_;

            const auto derivative = (err - err_old_) / dt;
            const auto D_term = get_kd() * derivative;

            real_t u = P_term + I_term + D_term;

            if (is_limited())
            {
                auto u_min = limits_.first;
                auto u_max = limits_.second;
                u = std::clamp(u, u_min, u_max);
            }

            err_old_ = err;
            return u;
        }
    }
}
