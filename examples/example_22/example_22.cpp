#include "bitrl/bitrl_types.h"
#include "bitrl/estimation/extended_kalman_filter.h"
#include "bitrl/utils/iteration_counter.h"
#include "bitrl/utils/io/csv_file_writer.h"
#include "bitrl/dynamics/diff_drive_dynamics.h"
#include "bitrl/dynamics/system_state.h"
#include "bitrl/utils/unit_converter.h"

#define BOOST_LOG_DYN_LINK
#include <boost/log/trivial.hpp>
#include <iostream>
#include <unordered_map>
#include <any>
#include <vector>
#include <random>

namespace example_2
{

using bitrl::real_t;
using bitrl::uint_t;
using bitrl::DynMat;
using bitrl::DynVec;
using bitrl::estimation::ExtendedKalmanFilter;

using bitrl::utils::io::CSVWriter;
using bitrl::utils::IterationCounter;
using bitrl::dynamics::DiffDriveDynamics;
using bitrl::dynamics::SysState;


class ObservationModel
{

public:

	// the ExtendedKalmanFilter expects an exposed
	// input_type
    typedef  DynVec<real_t> input_type;

    ObservationModel();

    // simply return the state
    const DynVec<real_t> evaluate(const DynVec<real_t>& input)const;

    // get the H or M matrix
    const DynMat<real_t>& get_matrix(const std::string& name)const;

private:

    DynMat<real_t> H;
    DynMat<real_t> M;
};

ObservationModel::ObservationModel()
    :
      H(2, 3),
      M(2, 2)
{
	H = DynMat<real_t>::Zero(2,3);
	M = DynMat<real_t>::Zero(2,2);
    H(0, 0) = 1.0;
    H(1,1) = 1.0;
    M(0,0) = 1.0;
    M(1, 1) = 1.0;

}

const DynVec<real_t>
ObservationModel::evaluate(const DynVec<real_t>& input)const{
    return input;
}

const DynMat<real_t>&
ObservationModel::get_matrix(const std::string& name)const{
    if(name == "H"){
        return H;
    }
    else if(name == "M"){
        return M;
    }

    throw std::logic_error("Invalid matrix name. Name "+name+ " not found");
}

const DynVec<real_t> get_measurement(const SysState<3>& state){
	
   DynVec<real_t> result(2);
   result[0] = state.get("X");
   result[1] = state.get("Y");
   return result;
}


}

int main() {

    using namespace example_2;
	
	BOOST_LOG_TRIVIAL(info)<<"Starting example...";

	uint_t n_steps = 300;
	auto time = 0.0;
	auto dt = 0.5;
	
	/// angular velocity
	auto w = 0.0;
	
	/// linear velocity
	auto vt = 1.0;
	
	std::array<real_t, 2> motion_control_error;
	motion_control_error[0] = 0.0;
	motion_control_error[1] = 0.0;
	
	DiffDriveDynamics exact_motion_model;
	exact_motion_model.set_matrix_update_flag(false);
	exact_motion_model.set_time_step(dt);
	
	DiffDriveDynamics motion_model;
	motion_model.set_time_step(dt);
	
	std::map<std::string, std::any> input;
	input["v"] = 1.0;
	input["w"] = 0.0;
	input["errors"] = motion_control_error;
	motion_model.initialize_matrices(input);
	
	ObservationModel observation;
	
	ExtendedKalmanFilter<DiffDriveDynamics, ObservationModel> ekf(motion_model, observation);
	
	DynMat<real_t> R = DynMat<real_t>::Zero(2, 2);
	R(0,0) = 1.0;
	R(1, 1) = 1.0;
	
	DynMat<real_t> Q = DynMat<real_t>::Zero(2, 2);
	Q(0,0) = 0.001;
	Q(1, 1) = 0.001;
	
	DynMat<real_t> P = DynMat<real_t>::Zero(3, 3);
	P(0, 0) = 1.0;
	P(1, 1) = 1.0;
	P(2, 2) = 1.0;
	
	ekf.with_matrix("P", P)
	   .with_matrix("R", R)
	   .with_matrix("Q", Q);
	   
	CSVWriter writer("state");
	writer.open();
	std::vector<std::string> names{"Time", "X_true", "Y_true", "Theta_true", "X", "Y", "Theta"};
	writer.write_column_names(names);

    try{
		
		
		BOOST_LOG_TRIVIAL(info)<<"Starting simulation... "<<time;
        uint_t counter=0;

        std::map<std::string, std::any> motion_input;
        motion_input["v"] = vt; // we keep the velocity constant
        motion_input["errors"] = motion_control_error;

        for(uint_t step=0; step < n_steps; ++step){

			BOOST_LOG_TRIVIAL(info)<<"Simulation time: "<<time;
            
            if(counter == 50){
              w = bitrl::utils::unit_converter::degrees_to_rad(45.0);
            }
            else if(counter == 100){
               w = bitrl::utils::unit_converter::degrees_to_rad(-45.0);
            }
            else if(counter == 150){
               w = bitrl::utils::unit_converter::degrees_to_rad(-45.0);
            }
            else{
                w = 0.0;
            }

            motion_input["w"] = w;
            
            auto& exact_state = exact_motion_model.evaluate(motion_input);

            ekf.predict(motion_input);

            auto& state = motion_model.get_state();
            auto z = get_measurement(state);
            ekf.update(z);

            BOOST_LOG_TRIVIAL(info)<<"Position: "<<ekf.get("X")<<", "<<ekf.get("Y");
            BOOST_LOG_TRIVIAL(info)<<"Orientation: (degrees)"<<bitrl::utils::unit_converter::rad_to_degrees(ekf.get("Theta"));
            BOOST_LOG_TRIVIAL(info)<<"V: "<<vt<<", W: "<<w;

            std::vector<real_t> row(7, 0.0);
			row[0] = time;
            row[1] = exact_state.get("X");
            row[2] = exact_state.get("Y");
			row[3] = exact_state.get("Theta");
            row[4] = state.get("X");
            row[5] = state.get("Y");
			row[6] = state.get("Theta");
            writer.write_row(row);

            time += dt;
            counter++;
        }
		
		BOOST_LOG_TRIVIAL(info)<<"Finished example...";
    }
    catch(std::runtime_error& e){
        std::cerr<<e.what()<<std::endl;
    }
    catch(std::logic_error& e){
        std::cerr<<e.what()<<std::endl;
    }
    catch(...){
        std::cerr<<"Unknown exception was raised whilst running simulation."<<std::endl;
    }
   
   return 0;
}
