#ifndef DIFF_DRIVE_DYNAMICS_H
#define DIFF_DRIVE_DYNAMICS_H

#include "bitrl/bitrl_types.h"
#include "bitrl/dynamics/system_state.h"
#include "bitrl/dynamics/motion_model_base.h"
#include "bitrl/dynamics/dynamics_matrix_descriptor.h"

#include <array>
#include <map>
#include <any>

namespace bitrl{
namespace dynamics{

///
/// \brief DiffDriveDynamics class. Describes the
/// motion dynamics of a differential drive system. It implements
/// the following equations
///
class DiffDriveDynamics: public MotionModelDynamicsBase<SysState<3>, 
                                                        DynamicsMatrixDescriptor, 
														std::map<std::string, std::any>>
{
public:

    ///
    /// \brief The DynamicVersion enum. Helper enum to
    /// disambiguate the two supplied configurations
    ///
    enum class DynamicVersion{V1, V2, V3};

    ///
    /// \brief The type of the state handled by this dynamics object
    ///
    typedef MotionModelDynamicsBase<SysState<3>,
                            DynamicsMatrixDescriptor,
                            std::map<std::string, std::any> >::state_type state_type;

    ///
    /// \brief input_t The type of the input for solving the dynamics
    ///
    typedef MotionModelDynamicsBase<SysState<3>,
                            DynamicsMatrixDescriptor,
                            std::map<std::string, std::any> >::input_type input_type;

    ///
    /// \brief matrix_t Matrix type that describes the dynamics
    ///
    typedef MotionModelDynamicsBase<SysState<3>,
                            DynamicsMatrixDescriptor,
                            std::map<std::string, std::any> >::matrix_type matrix_type;

    ///
    /// \brief vector_t
    ///
    typedef MotionModelDynamicsBase<SysState<3>,
                            DynamicsMatrixDescriptor,
                            std::map<std::string, std::any> >::vector_type vector_type;

    ///
    /// \brief integrate_state_v1
    ///
    static SysState<3> integrate_state_v1(const SysState<3>& state, real_t tol, real_t dt,
                                                real_t v, real_t w, const std::array<real_t, 2>& errors);


    ///
    /// \brief integrate_state_v2
    ///
    static SysState<3> integrate_state_v2(const SysState<3>& state, real_t dt,
                                                real_t v, real_t w, const std::array<real_t, 2>& errors);

    ///
    /// \brief integrate_state_v3
    ///
    static SysState<3> integrate_state_v3(const SysState<3>& state, real_t r, real_t l, real_t dt, real_t w1,
                                                real_t w2, const std::array<real_t, 2>& errors);

    ///
    /// \brief integrate Factory method to apply
    /// the different integration methods
    ///
    static SysState<3> integrate(const SysState<3>& state, 
	                             const input_type& input, const DynamicVersion version);

    ///
    /// \brief Constructor
    ///
    explicit DiffDriveDynamics(DynamicVersion type = DynamicVersion::V1, 
	                           bool update_description_matrices_on_evaluate = true);

    ///
    /// \brief Constructor
    ///
    explicit DiffDriveDynamics(state_type&& state);

    ///
    /// \brief Evaluate the new state using the given input
    /// it also updates the various matrices if needed
    ///
    virtual state_type& evaluate(const input_type& input )override;

    ///
    /// \brief Integrate the new state. It also uses error terms
    ///
    void integrate(const input_type& input);

    ///
    /// \brief Read the x-coordinate
    ///
    real_t get_x_position()const{return this->state_.get("X");}

    ///
    /// \brief Set the x-coordinate
    ///
    void set_x_position(real_t x){this->state_.set("X", x);}

    ///
    /// \brief Read the y-coordinate
    ///
    real_t get_y_position()const{return this->state_.get("Y");}

    ///
    /// \brief Set the y-coordinate
    ///
    void set_y_position(real_t y){this->state_.set("Y", y);}

    ///
    /// \brief Read the y-coordinate
    ///
    real_t get_orientation()const{return this->state_.get("Theta");}

    ///
    /// \brief Set the orientation
    ///
    void set_orientation(real_t theta){this->state_.set("Theta", theta);}

    ///
    /// \brief get_velocity Returns the velocity used for integration
    ///
    real_t get_velocity()const{return v_;}

    ///
    /// \brief get_velocity Returns the angular velocity used for integration
    ///
    real_t get_angular_velocity()const{return w_;}

    ///
    /// \brief updates the matrices used to describe this
    /// motion model
    ///
    void update_matrices(const input_type& input);

    ///
    /// \brief Initialize the matrices describing the
    /// the dynamics
    ///
    void initialize_matrices(const input_type& input);

private:

    ///
    /// \brief v_ The velocity used for integration. Updated
    /// every time the application calls integrate
    ///
    real_t v_;

    ///
    /// \brief w_ The angular velocity used for integration
    ///
    real_t w_;

    ///
    /// \brief
    ///
    DynamicVersion type_;

};

}

}

#endif // DIFF_DRIVE_DYNAMICS_H
