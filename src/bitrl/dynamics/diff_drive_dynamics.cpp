#include "bitrl/dynamics/diff_drive_dynamics.h"
#include "bitrl/bitrl_consts.h"
#include "bitrl/utils/maths/math_utils.h"
#include "bitrl/utils/std_map_utils.h"

#include <cmath>
#include <iostream>

namespace bitrl
{
namespace dynamics
{

SysState<3> DiffDriveDynamics::integrate_state_v1(const SysState<3> &state, real_t tol, real_t dt,
                                                  real_t v, real_t w,
                                                  const std::array<real_t, 2> &errors)
{

    SysState<3> other(state);
    auto values = other.get_values();

    if (std::fabs(w) < tol)
    {

        /// assume zero angular velocity
        auto distance = 0.5 * v * dt;
        auto xincrement = (distance + errors[0]) * std::cos(values[2] + errors[1]);
        auto yincrement = (distance + errors[0]) * std::sin(values[2] + errors[1]);

        other[0] += xincrement;
        other[1] += yincrement;
    }
    else
    {

        other[2] += w * dt + errors[1];

        /// clip the value
        if (std::fabs(state[2]) > bitrl::consts::maths::PI)
        {
            other[2] = bitrl::utils::maths::sign(state[2]) * bitrl::consts::maths::PI;
        }

        other[0] += ((v / (2.0 * w)) + errors[0]) * (std::sin(state[2]) - std::sin(values[2]));
        other[1] -= ((v / (2.0 * w)) + errors[0]) * (std::cos(state[2]) - std::cos(values[2]));
    }

    return other;
}

SysState<3> DiffDriveDynamics::integrate_state_v2(const SysState<3> &state, real_t dt, real_t v,
                                                  real_t w, const std::array<real_t, 2> &errors)
{

    auto values = state.get_values();
    auto distance = v * dt; // 0.5*v*dt;
    auto xincrement = (distance + errors[0]) * std::cos(values[2] + errors[1]);
    auto yincrement = (distance + errors[0]) * std::sin(values[2] + errors[1]);

    SysState<3> other(state);

    other[0] += xincrement;
    other[1] += yincrement;
    other[2] += dt * w;
    return other;
}

SysState<3> DiffDriveDynamics::integrate_state_v3(const SysState<3> &state, real_t r, real_t l,
                                                  real_t dt, real_t w1, real_t w2,
                                                  const std::array<real_t, 2> &errors)
{

    auto values = state.get_values();
    auto xincrement = dt * 0.5 * r * (w1 + w2 + errors[0]) * std::cos(values[2] + errors[1]);
    auto yincrement = dt * 0.5 * r * (w1 + w2 + errors[0]) * std::sin(values[2] + errors[1]);

    SysState<3> other(state);
    other[0] += xincrement;
    other[1] += yincrement;
    other[2] += dt * r * (w1 - w2) / (2.0 * l);
    return other;
}

SysState<3> DiffDriveDynamics::integrate(const SysState<3> &state,
                                         const DiffDriveDynamics::input_type &input,
                                         const DynamicVersion version)
{

    auto result = state;
    if (version == DiffDriveDynamics::DynamicVersion::V1)
    {

        auto w = bitrl::utils::template resolve<real_t>("w", input);
        auto v = bitrl::utils::template resolve<real_t>("v", input);
        auto errors = bitrl::utils::template resolve<std::array<real_t, 2>>("errors", input);
        auto dt = bitrl::utils::template resolve<real_t>("dt", input);
        auto tol = bitrl::utils::template resolve<real_t>("tol", input);
        result = DiffDriveDynamics::integrate_state_v1(state, tol, dt, v, w, errors);
    }
    else if (version == DiffDriveDynamics::DynamicVersion::V2)
    {

        auto w = bitrl::utils::template resolve<real_t>("w", input);
        auto v = bitrl::utils::template resolve<real_t>("v", input);
        auto errors = bitrl::utils::template resolve<std::array<real_t, 2>>("errors", input);
        auto dt = bitrl::utils::template resolve<real_t>("dt", input);
        result = DiffDriveDynamics::integrate_state_v2(state, dt, v, w, errors);
    }
    else if (version == DiffDriveDynamics::DynamicVersion::V3)
    {

        // in this scenario we have the wheels speed as input

        auto w1 = bitrl::utils::template resolve<real_t>("w1", input);
        auto w2 = bitrl::utils::template resolve<real_t>("w2", input);
        auto errors = bitrl::utils::template resolve<std::array<real_t, 2>>("errors", input);
        auto r = bitrl::utils::template resolve<real_t>("r", input);
        auto l = bitrl::utils::template resolve<real_t>("l", input);
        auto dt = bitrl::utils::template resolve<real_t>("dt", input);
        result = DiffDriveDynamics::integrate_state_v3(state, r, l, dt, w1, w2, errors);
    }

    return result;
}

DiffDriveDynamics::DiffDriveDynamics(DiffDriveDynamics::DynamicVersion type,
                                     bool update_description_matrices_on_evaluate)
    : MotionModelDynamicsBase<SysState<3>, DynamicsMatrixDescriptor,
                              std::map<std::string, std::any>>(
          update_description_matrices_on_evaluate),
      v_(0.0), w_(0.0), type_(type)
{
    this->state_.set(0, {"X", 0.0});
    this->state_.set(1, {"Y", 0.0});
    this->state_.set(2, {"Theta", 0.0});
}

DiffDriveDynamics::DiffDriveDynamics(DiffDriveDynamics::state_type &&state)
    : MotionModelDynamicsBase<SysState<3>, DynamicsMatrixDescriptor,
                              std::map<std::string, std::any>>(),
      v_(0.0), w_(0.0)
{
    this->state_ = state;
}

void DiffDriveDynamics::integrate(const DiffDriveDynamics::input_type &input)
{

    // before we do the integration
    // update the matrices
    if (this->allows_matrix_updates())
    {
        update_matrices(input);
    }

    if (type_ == DiffDriveDynamics::DynamicVersion::V1)
    {

        auto w = bitrl::utils::template resolve<real_t>("w", input);
        auto v = bitrl::utils::template resolve<real_t>("v", input);
        auto errors = bitrl::utils::template resolve<std::array<real_t, 2>>("errors", input);
        auto result = DiffDriveDynamics::integrate_state_v1(this->state_, tol_, get_time_step(), v,
                                                            w, errors);
        this->state_ = result;

        // update the velocities and angular velocities
        v_ = v;
        w_ = w;
    }
    else if (type_ == DiffDriveDynamics::DynamicVersion::V2)
    {

        auto w = bitrl::utils::template resolve<real_t>("w", input);
        auto v = bitrl::utils::template resolve<real_t>("v", input);
        auto errors = bitrl::utils::template resolve<std::array<real_t, 2>>("errors", input);
        auto result =
            DiffDriveDynamics::integrate_state_v2(this->state_, get_time_step(), v, w, errors);
        this->state_ = result;

        // update the velocities and angular
        // velocities
        v_ = v;
        w_ = w;
    }
    else if (type_ == DiffDriveDynamics::DynamicVersion::V3)
    {

        // in this scenario we have the wheels speed as input

        auto w1 = bitrl::utils::template resolve<real_t>("w1", input);
        auto w2 = bitrl::utils::template resolve<real_t>("w2", input);
        auto errors = bitrl::utils::template resolve<std::array<real_t, 2>>("errors", input);
        auto r = bitrl::utils::template resolve<real_t>("r", input);
        auto l = bitrl::utils::template resolve<real_t>("l", input);
        auto result = DiffDriveDynamics::integrate_state_v3(this->state_, r, l, get_time_step(), w1,
                                                            w2, errors);
        this->state_ = result;

        v_ = 0.5 * r * (w1 + w2);
        w_ = r * (w1 - w2) / (2.0 * l);
    }
}

DiffDriveDynamics::state_type &
DiffDriveDynamics::evaluate(const DiffDriveDynamics::input_type &input)
{
    integrate(input);
    return this->state_;
}

void DiffDriveDynamics::initialize_matrices(const DiffDriveDynamics::input_type &input)
{

    // if we initialize the matrices
    // then we should set the matrix update flag to true
    set_matrix_update_flag(true);

    if (!this->has_matrix("F"))
    {
        matrix_type F = matrix_type::Zero(3, 3); //(3,3, 0.0);
        this->set_matrix("F", F);
    }

    if (!this->has_matrix("L"))
    {
        matrix_type L = matrix_type::Zero(3, 2); //, 0.0);
        this->set_matrix("L", L);
    }

    update_matrices(input);
}

void DiffDriveDynamics::update_matrices(const DiffDriveDynamics::input_type &input)
{

    auto w = bitrl::utils::template resolve<real_t>("w", input);
    auto v = bitrl::utils::template resolve<real_t>("v", input);
    auto errors = bitrl::utils::template resolve<std::array<real_t, 2>>("errors", input);

    auto distance = 0.5 * v * get_time_step();
    auto orientation = w * get_time_step();
    auto values = this->state_.get_values();

    if (std::fabs(w) < tol_)
    {

        auto &F = this->get_matrix("F");

        F(0, 0) = 1.0;
        F(0, 1) = 0.0;
        F(0, 2) = (distance + errors[0]) * std::sin(values[2] + orientation + errors[1]);

        F(1, 0) = 0.0;
        F(1, 1) = 1.0;
        F(1, 2) = -(distance + errors[0]) * std::cos(values[2] + orientation + errors[1]);

        F(2, 0) = 0.0;
        F(2, 1) = 0.0;
        F(2, 2) = 1.0;

        auto &L = this->get_matrix("L");

        L(0, 0) = std::cos(values[2] + orientation + errors[1]);
        L(0, 1) = (distance + errors[0]) * std::sin(values[2] + orientation + errors[1]);

        L(1, 0) = std::sin(values[2] + orientation + errors[1]);
        L(1, 1) = -(distance + errors[0]) * std::cos(values[2] + orientation + errors[1]);

        L(2, 0) = 0.0;
        L(2, 1) = 1.0;
    }
    else
    {

        auto &F = this->get_matrix("F");

        F(0, 0) = 1.0;
        F(0, 1) = 0.0;
        F(0, 2) = -(distance + errors[0]) * std::cos(values[2] + orientation + errors[1]) +
                  (distance + errors[0]) * std::cos(values[2]);

        F(1, 0) = 0.0;
        F(1, 1) = 1.0;
        F(1, 2) = -(distance + errors[0]) * std::sin(values[2] + orientation + errors[1]) +
                  (distance + errors[0]) * std::sin(values[2]);

        F(2, 0) = 0.0;
        F(2, 1) = 0.0;
        F(2, 2) = 1.0;

        auto &L = this->get_matrix("L");

        L(0, 0) = std::sin(values[2] + orientation + errors[1]) - std::sin(values[2]);

        L(0, 1) = -((v / 2.0 * w) + errors[0]) * std::cos(values[2] + orientation + errors[1]) *
                  std::sin(values[2] + orientation + errors[1]);

        L(1, 0) = -std::cos(values[2] + orientation + errors[1]) + std::cos(values[2]);
        L(1, 1) = ((v / 2.0 * w) + errors[0]) * std::sin(values[2] + orientation + errors[1]);

        L(2, 0) = 0.0;
        L(2, 1) = 1.0;
    }
}

} // namespace dynamics
} // namespace bitrl
