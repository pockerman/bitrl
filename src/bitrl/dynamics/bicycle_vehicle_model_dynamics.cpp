#include "bitrl/dynamics/bicycle_vehicle_model_dynamics.h"
#include "bitrl/utils/std_map_utils.h"

namespace bitrl
{
namespace dynamics
{

BicycleVehicleModelDynamics::BicycleVehicleModelDynamics(
    const BicycleVehicleModelDynamicsConfig config, SysState<3> &init_state)
    : BicycleVehicleModelDynamics::base_type(init_state), config_(config)
{
}

void BicycleVehicleModelDynamics::integrate(const BicycleVehicleModelDynamics::input_type &input)
{

    auto old_x = this->state_.get("X");
    auto old_y = this->state_.get("Y");
    auto old_theta = this->state_.get("Theta");
    auto dt = config_.dt;

    auto v = bitrl::utils::template resolve<real_t>("v", input);
    auto delta = bitrl::utils::template resolve<real_t>("delta", input);

    // the input error for v and delta
    auto errors = bitrl::utils::template resolve<std::array<real_t, 2>>("errors", input);

    /// before we do the integration
    /// update the matrices
    //    if(this->allows_matrix_updates()){
    //
    //      DiffDriveDynamics::input_t input(v,w,errors);
    //      update_matrices(input);
    //    }

    auto x_new = old_x + (v + errors[0]) * std::cos(old_theta) * dt;
    auto y_new = old_y + (v + errors[0]) * std::sin(old_theta) * dt;
    auto theta_new = old_theta * (v + errors[0]) * std::tan(delta + errors[1]) * dt / config_.L;

    this->state_[0] = x_new;
    this->state_[1] = y_new;
    this->state_[2] = theta_new; // we may want to clip this value
}

BicycleVehicleModelDynamics::state_type &
BicycleVehicleModelDynamics::evaluate(const BicycleVehicleModelDynamics::input_type &input)
{
    integrate(input);
    return this->get_state();
}

void BicycleVehicleModelDynamics::initialize_matrices(
    const BicycleVehicleModelDynamics::input_type &)
{
}

void BicycleVehicleModelDynamics::update_matrices(
    const BicycleVehicleModelDynamics::input_type &input)
{
}

} // namespace dynamics
} // namespace bitrl
