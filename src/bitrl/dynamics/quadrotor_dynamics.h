#ifndef QUADROTOR_DYNAMICS_H
#define QUADROTOR_DYNAMICS_H

#include "bitrl/bitrl_types_v2.h"
#include "bitrl/dynamics/system_state.h"
#include "bitrl/dynamics/motion_model_base.h"
#include "bitrl/dynamics/dynamics_matrix_descriptor.h"
#include "bitrl/dynamics/system_state.h"

#include <any>


namespace bitrl::dynamics {
	
	
struct QuadrotorDynamicsConfig
{
	///
	/// \brief flag indicating if gravity is used
	///
	bool use_gravity{true};
	
	///
	///
	///
	bool use_description_matrices{false};
	
	///
	/// \brief mass
	///
	real_t mass;
	
	///
	/// \brief the arm length
	///
	real_t l;
	
	///
	/// \brief Proportionality constant
	/// to scale the morot velocity
	///
	real_t k_1;
	
	///
	/// \brief Proportionality constant
	/// to scale the morot velocity used when
	/// calculating torques
	///
	real_t k_2;

	///
	/// \brief The time step to ise
	///
	real_t dt;
	
	///
	/// \brief
	///
	real_t Jx;
	
	///
	/// \brief
	///
	real_t Jy;
	
	///
	/// \brief
	///
	real_t Jz;
};

///
/// \brief The QuadrotorDynamics class. Implements quadrotor dynamics
/// The implementation of this class follows the System Modeling
/// section at https://wilselby.com/research/arducopter/modeling/
/// and https://scholarsarchive.byu.edu/cgi/viewcontent.cgi?article=2324&context=facpub
/// The quadrotor is assumed to have four rotors 
/// Each rotor consists of a brushless DC motor and rotor with a fixed pitch.
/// It is assumed that the quadrotor can move in 6 degrees of freedom; 3-translational and 3-rotational, 
/// The motion of the quadrotor is controlled via 4 inputs namely the speeds of the 4 motors.
/// The quadrotor model that this class implements assumes the following:
///
/// - The quadrotor structure is rigid and symmetrical with a center of mass aligned with the center of the body frame of the vehicle.
/// - The thrust and drag of each motor is proportional to the square of the motor velocity.
/// - The propellers are considered to be rigid and therefore blade flapping is negligible (deformation of propeller blades due to high velocities and flexible material).
/// - The Earth is flat and non-rotating (difference of gravity by altitude or the spin of the earth is negligible)
/// - Surrounding fluid velocities (wind) are negligible.
/// - Ground effect is negligible
///
/// The class integrates the translational and rotational dynamics equations
/// on the body frame
///
class QuadrotorDynamics: public MotionModelDynamicsBase<SysState<12>,
                                                        DynamicsMatrixDescriptor, 
														RealVec>
{
public:
	
	typedef MotionModelDynamicsBase<SysState<12>,
									DynamicsMatrixDescriptor, 
									RealVec> base_type;

    ///
    /// \brief The type of the state handled by this dynamics object
    ///
    typedef base_type::state_type state_type;

    ///
    /// \brief input_t The type of the input for solving the dynamics
    ///
    typedef base_type::input_type input_type;

    ///
    /// \brief matrix_t Matrix type that describes the dynamics
    ///
    typedef base_type::matrix_type matrix_type;

    ///
    /// \brief vector_t
    ///
    typedef base_type::vector_type vector_type;

    ///
    /// \brief QuadrotorDynamics Constructor. 
	/// Specify the configuration parameters of the
	/// simulated quadrotor. It is assumed that
	/// the mass of the quadrotor remains fixed
	/// and the system state to be tracked
    ///
    QuadrotorDynamics(QuadrotorDynamicsConfig config,
					  SysState<12>& state);

    ///
    /// \brief Evaluate the new state using the given input
    /// it also updates the various matrices if needed
    ///
    virtual state_type& evaluate(const input_type& input )override;

    ///
    /// \brief Integrate the new state. It also uses error terms
    ///
    void integrate(const RealVec& motor_w);
	
	///
	/// \brief Implements the translational dynamics.
	/// It accepts the motors angular velocities.
	/// Notice that this model will squared and input
	/// velocities
	///
	void translational_dynamics(const RealVec& motor_w);
	
	///
	/// \brief Implements the translational dynamics.
	/// It accepts the motors angular velocities.
	/// Notice that this model will squared and input
	/// velocities
	///
	void rotational_dynamics(const RealVec& motor_w);
	
	///
	/// \brief Returns the current position NED frame.
	/// Note that the z-coordinate has to be multiplied with -1
	/// in order to represent an upwards looking z-axis
	///
	RealColVec3d get_position()const{return get_position_from_state_();}
	
	///
	/// \brief Returns the current linear velocity (body frame)
	///
	RealColVec3d get_velocity()const{return get_velocity_from_state_();}
	
	///
	/// \brief Returns the current angular velocity (body frame)
	///
	RealColVec3d get_angular_velocity()const{return get_angular_velocity_from_state_();}
	
	///
	/// \brief Returns the Euler angles (0: phi, 1: theta, 2:psi)
	///
	RealColVec3d get_euler_angles()const{return get_euler_angles_from_state_();}
	
	///
	/// \brief The phi Euler angle in rad
	///
	real_t phi()const noexcept {return get_euler_angles_from_state_()[0];}
	
	///
	/// \brief The theta Euler angle in rad
	///
	real_t theta()const noexcept {return get_euler_angles_from_state_()[1];}
	
	///
	/// \brief The psi Euler angle in rad
	///
	real_t psi()const noexcept {return get_euler_angles_from_state_()[2];}
	
private:
	
	///
	/// \brief Configuration of the dynamics integrator
	///
	QuadrotorDynamicsConfig config_;
	
	///
	/// \brief The rotation matrix from body
	/// to inertial frames
	///
	RealMat3d rotation_mat_;
	
	///
	/// \brief The rotation matrix from body
	/// to inertial frames
	///
	RealMat3d euler_mat_;
	
	///
	/// \brief Track the velocity time gradient
	///
	RealColVec3d v_dot_;
	
	///
	/// \brief Track the angular velocity time gradient
	///
	RealColVec3d omega_dot_;
	
	///
	/// \brief The time derivative of the euler angles
	///
	RealColVec3d euler_dot_;
	
	///
	/// Update the position
	///
	void update_position_();
	
	///
	/// \brief Update the rotation matrix
	///
	void update_rotation_matrix_();
	
	void update_euler_angles_();
	
	
	RealColVec3d get_velocity_from_state_()const;
	RealColVec3d get_angular_velocity_from_state_()const;
	RealColVec3d get_position_from_state_()const;
	RealColVec3d get_euler_angles_from_state_()const;
	
	
	
};

inline
RealColVec3d 
QuadrotorDynamics::get_velocity_from_state_()const{
	
	RealColVec3d v = RealColVec3d::Zero();
	v[0] = this -> get_state_property("u");
	v[1] = this -> get_state_property("v");
	v[2] = this -> get_state_property("w");
	return v;
	
}

inline
RealColVec3d 
QuadrotorDynamics::get_position_from_state_()const{
	
	RealColVec3d v = RealColVec3d::Zero();
	v[0] = this -> get_state_property("x");
	v[1] = this -> get_state_property("y");
	v[2] = this -> get_state_property("z");
	return v;
	
}

inline
RealColVec3d 
QuadrotorDynamics::get_angular_velocity_from_state_()const{
	
	RealColVec3d v = RealColVec3d::Zero();
	v[0] = this -> get_state_property("p");
	v[1] = this -> get_state_property("q");
	v[2] = this -> get_state_property("r");
	return v;
}

inline
RealColVec3d 
QuadrotorDynamics::get_euler_angles_from_state_()const{
	
	RealColVec3d v = RealColVec3d::Zero();
	v[0] = this -> get_state_property("phi");
	v[1] = this -> get_state_property("theta");
	v[2] = this -> get_state_property("psi");
	return v;
}

}



#endif // QUADROTOR_DYNAMICS_H
