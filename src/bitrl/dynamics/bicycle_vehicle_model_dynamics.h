#ifndef BICYCLE_VEHICLE_MODEL_DYNAMICS_H
#define BICYCLE_VEHICLE_MODEL_DYNAMICS_H

#include "bitrl/bitrl_types_v2.h"
#include "bitrl/dynamics/system_state.h"
#include "bitrl/dynamics/motion_model_base.h"
#include "bitrl/dynamics/dynamics_matrix_descriptor.h"

#include <any>
#include <map>
#include <string>

namespace bitrl{
namespace dynamics{
	
struct BicycleVehicleModelDynamicsConfig
{
	///
	/// \brief The time step to use in secs
	///
	real_t dt;
	
	///
	/// \brief The distance between the front and
	/// real wheels in m
	///
	real_t L;
};


///
/// \brief The BicycleVehicleModelDynamics class.
/// Implements the bicycle vehicle model dynamics.
/// The implementation of the dynamics is taken
/// from the paper: Deep Reinforcement Learning for Mobile Robot Path Planning
/// by Hao Liu, Yi Shen, Shuangjiang Yu, Zijun Gao, Tong Wu
/// The state of the model is described by the tuple (X, Y, Phi)
/// where (X,Y) denotes the 2D position of the vehicle in the world coordinates
/// and phi denotes the heading of the vehicle
///
class BicycleVehicleModelDynamics: public MotionModelDynamicsBase<SysState<3>, 
                                                                  DynamicsMatrixDescriptor,
                                                                  std::map<std::string, std::any>>
{

public:
    typedef MotionModelDynamicsBase<SysState<3>, 
	                                DynamicsMatrixDescriptor,
                                    std::map<std::string, std::any>> base_type;

    typedef base_type::state_type state_type;
    typedef base_type::input_type input_type;
    typedef base_type::matrix_descriptor_type matrix_descriptor_type;
    typedef typename matrix_descriptor_type::matrix_type matrix_type;
    typedef typename matrix_descriptor_type::vector_type vector_type;

    ///
    /// \brief Constructor
    ///
    BicycleVehicleModelDynamics(const BicycleVehicleModelDynamicsConfig config,
								SysState<3>& init_state);

    ///
    /// \brief integrate. Integrate the dynamics equations
	/// the model expects the following inputs
	/// the velocity of the vehicle v and the angle delta of the front wheel
    ///
    void integrate(const input_type& input );

    ///
    /// \brief evaluate
    /// \param input
    /// \return
    ///
    state_type& evaluate(const input_type& input );

    ///
    /// \brief initialize_matrices. Initialize the matrix description of the
    /// dynamics
    ///
    void initialize_matrices(const input_type& input);

    ///
    /// \brief update_matrices. Update the matrix description of the
    /// dynamics
    ///
    void update_matrices(const input_type& input);

    ///
    /// \brief load_from_json Load the description of the chassis from
    /// the given json file
    ///
    void load_from_json(const std::string& filename){};

private:
	
	///
	/// \brief Configuration of the dynamics integrator
	///
	BicycleVehicleModelDynamicsConfig config_;

};

}
}

#endif // BICYCLE_VEHICLE_MODEL_DYNAMICS_H
