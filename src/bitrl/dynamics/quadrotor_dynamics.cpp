#include "bitrl/dynamics/quadrotor_dynamics.h"
#include "bitrl/bitrl_consts.h"
#include "bitrl/utils/maths/math_utils.h"

#include <Eigen/Geometry> // need this for cross otherwise we get linking errors
#include <cmath>

namespace bitrl
{
namespace dynamics
{

QuadrotorDynamics::QuadrotorDynamics(QuadrotorDynamicsConfig config, SysState<12> &state)
    : MotionModelDynamicsBase<SysState<12>, DynamicsMatrixDescriptor, RealVec>(state),
      config_(config)
{

    v_dot_ = RealColVec3d::Zero(3);
    omega_dot_ = RealColVec3d::Zero(3);
    euler_dot_ = RealColVec3d::Zero(3);

    rotation_mat_ = RealMat3d::Zero();
    euler_mat_ = RealMat3d::Zero();
}

QuadrotorDynamics::state_type &QuadrotorDynamics::evaluate(const input_type &input)
{
    integrate(input);
    return this->get_state();
}

void QuadrotorDynamics::integrate(const RealVec &motor_w)
{

    translational_dynamics(motor_w);
    rotational_dynamics(motor_w);

    update_euler_angles_();
    update_rotation_matrix_();
    update_position_();
}

void QuadrotorDynamics::update_position_()
{

    const auto dt = config_.dt;
    const auto old_p = get_position_from_state_();
    const auto old_v = get_velocity_from_state_();
    RealColVec3d p = dt * rotation_mat_ * old_v + old_p;

    // update the position
    this->get_state().set("x", p[0]);
    this->get_state().set("y", p[1]);
    this->get_state().set("z", p[2]);
}

void QuadrotorDynamics::update_euler_angles_()
{

    RealColVec3d euler_angles_old = get_euler_angles_from_state_();
    RealColVec3d old_omega = get_angular_velocity_from_state_();
    const auto dt = config_.dt;
    const auto phi = euler_angles_old[0];
    const auto theta = euler_angles_old[1];
    const auto psi = euler_angles_old[2];

    euler_mat_(0, 0) = 1.0;
    euler_mat_(0, 1) = std::sin(phi) * std::tan(theta);
    euler_mat_(0, 2) = std::cos(phi) * std::tan(theta);

    euler_mat_(1, 0) = 0.0;
    euler_mat_(1, 1) = std::cos(phi);
    euler_mat_(1, 2) = -std::sin(phi);

    euler_mat_(2, 0) = 0.0;
    euler_mat_(2, 1) = std::sin(phi) / std::cos(theta);
    euler_mat_(2, 2) = std::cos(phi) / std::cos(theta);

    RealColVec3d euler = dt * euler_mat_ * old_omega + euler_angles_old;

    euler_dot_ = (euler - euler_angles_old) / dt;

    this->set_state_property("phi", euler[0]);
    this->set_state_property("theta", euler[1]);
    this->set_state_property("psi", euler[2]);
}

void QuadrotorDynamics::update_rotation_matrix_()
{

    RealColVec3d euler_angles_old = get_euler_angles_from_state_();
    const auto phi = euler_angles_old[0];
    const auto theta = euler_angles_old[1];
    const auto psi = euler_angles_old[2];

    auto c_theta = std::cos(theta);
    auto c_psi = std::cos(psi);
    auto c_phi = std::cos(phi);

    auto s_theta = std::sin(theta);
    auto s_psi = std::sin(psi);
    auto s_phi = std::sin(phi);

    rotation_mat_(0, 0) = c_theta * c_psi;
    rotation_mat_(0, 1) = s_psi * s_theta * c_psi - c_phi * s_psi;
    rotation_mat_(0, 2) = c_phi * s_theta * c_psi + s_phi * s_psi;

    rotation_mat_(1, 0) = c_theta * s_psi;
    rotation_mat_(1, 1) = s_phi * s_theta * s_psi + c_phi * c_psi;
    rotation_mat_(1, 2) = c_phi * s_theta * s_psi - s_phi * c_psi;

    rotation_mat_(2, 0) = s_theta;
    rotation_mat_(2, 1) = -s_phi * c_theta;
    rotation_mat_(2, 2) = -c_phi * c_theta;
}

void QuadrotorDynamics::rotational_dynamics(const RealVec &motor_w)
{

    const auto l = config_.l;
    const auto k_1 = config_.k_1;
    const auto k_2 = config_.k_2;
    const auto dt = config_.dt;

    RealColVec3d old_omega = get_angular_velocity_from_state_();

    auto p = old_omega[0];
    auto q = old_omega[1];
    auto r = old_omega[2];

    auto Jx = config_.Jx;
    auto Jy = config_.Jy;
    auto Jz = config_.Jz;

    RealColVec3d omega_cross_h = RealColVec3d::Zero(3);
    omega_cross_h[0] = ((Jy - Jz) / Jx) * q * r;
    omega_cross_h[1] = ((Jz - Jx) / Jy) * p * r;
    omega_cross_h[2] = ((Jx - Jy) / Jz) * p * q;

    // the torque vector
    RealColVec3d tau = RealColVec3d::Zero(3);

    // tau_phi = l*K_T*((omeag_4)^2 - (omeag_2)^2)
    tau[0] =
        l * k_1 * (bitrl::utils::maths::sqr(motor_w[3]) - bitrl::utils::maths::sqr(motor_w[1]));
    tau[0] *= 1.0 / Jx;

    // tau_theta = l*K_T*((omeag_1)^2 - (omeag_3)^2)
    tau[1] =
        l * k_1 * (bitrl::utils::maths::sqr(motor_w[0]) - bitrl::utils::maths::sqr(motor_w[2]));
    tau[1] = 1.0 / Jy;

    // tau_psi = Kd((omega_1)^2 - (omega_2)^2 + (omega_3)^2 - (omega_4)^2)
    tau[2] = k_2 * (bitrl::utils::maths::sqr(motor_w[0]) - bitrl::utils::maths::sqr(motor_w[1]) +
                    bitrl::utils::maths::sqr(motor_w[2]) - bitrl::utils::maths::sqr(motor_w[3]));
    tau[2] *= 1.0 / Jz;

    RealColVec3d omega = dt * omega_cross_h + dt * tau - old_omega;

    omega_dot_ = (omega - old_omega) / dt;

    // update the angular velocity
    this->set_state_property("p", omega[0]);
    this->set_state_property("q", omega[1]);
    this->set_state_property("r", omega[2]);
}

void QuadrotorDynamics::translational_dynamics(const RealVec &motor_w)
{

    const auto mass = config_.mass;
    const auto dt = config_.dt;

    // compute the gravity force
    RealColVec3d fg = RealColVec3d::Zero(3);

    if (config_.use_gravity)
    {

        const auto phi_ = phi();
        const auto theta_ = theta();

        // we do not contain the mass constant so don't use it below
        fg[0] = -bitrl::consts::maths::G * std::sin(theta_);
        fg[1] = bitrl::consts::maths::G * std::cos(theta_) * std::sin(phi_);
        fg[2] = bitrl::consts::maths::G * std::cos(theta_) * std::cos(phi_);
    }

    // compute the total thurst force
    RealColVec3d ft = RealColVec3d::Zero(3);
    ft[2] = config_.k_1 * bitrl::utils::maths::sum_sqr(motor_w);

    RealColVec3d old_v = get_velocity_from_state_();
    RealColVec3d old_omega = get_angular_velocity_from_state_();

    // form the vector ωb/i × v
    RealColVec3d omega_cross_v = old_omega.cross(old_v);

    // compute the velocity increment.
    // Note that the mass is dropped when accounting for gravity
    // as fg does not contain it
    RealColVec3d v = dt * omega_cross_v + dt * fg - (1.0 / mass) * dt * ft - old_v;

    // compute the time velocity derivative
    v_dot_ = (v - old_v) / dt;

    // update the old velocity
    this->set_state_property("u", v[0]);
    this->set_state_property("v", v[1]);
    this->set_state_property("w", v[2]);
}

} // namespace dynamics
} // namespace bitrl
