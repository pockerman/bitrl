/**
  * KalmanFilter example. The example is taken from the 
  * paper <a href="https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf">An Introduction to the Kalman Filter</a> by
  * Greg Welch and Gary Bishop
  * */

#include "bitrl/bitrl_types.h"
#include "bitrl/estimation/kalman_filter.h"
#include "bitrl/estimation/kf_model_base.h"
#include "bitrl/utils/iteration_counter.h"
#include "bitrl/utils/io/csv_file_writer.h"

#define BOOST_LOG_DYN_LINK
#include <boost/log/trivial.hpp>
#include <iostream>
#include <unordered_map>
#include <any>
#include <vector>
#include <random>

namespace example_1
{

using bitrl::real_t;
using bitrl::uint_t;
using bitrl::DynMat;
using bitrl::DynVec;
using bitrl::estimation::KalmanFilterConfig;
using bitrl::estimation::KalmanFilter;
using bitrl::estimation::KFMotionModelBase;
using bitrl::estimation::KFModelBase;
using bitrl::utils::IterationCounter;
using bitrl::utils::io::CSVWriter;


real_t DT = 1.0;
real_t SIM_TIME = 50.0;

struct Cmd
{
	// v: [m/s] omega: [rad/s]
	static DynVec<real_t> cmd();
};

DynVec<real_t> 
Cmd::cmd(){
	
	DynVec<real_t> u(1);
	u << 0.0;
    return u;
}

// simple struct that describes the Motion of the robots
struct MotionModel: public KFMotionModelBase<DynMat<real_t>, DynVec<real_t>>
{
	const static uint_t VEC_SIZE;
	
};

const uint_t MotionModel::VEC_SIZE = 1;

// simple struct that describes the Observation model
struct ObservationModel: public KFModelBase<DynMat<real_t>>
{
	const static uint_t VEC_SIZE;
	static DynVec<real_t> sensors();
};

const uint_t ObservationModel::VEC_SIZE = 1;

const real_t STD = 0.1;
const real_t MU = -0.37727;

DynVec<real_t> 
ObservationModel::sensors(){ 
	DynVec<real_t> all_sensors(1);
	
	std::normal_distribution<real_t> d{MU , STD};
	std::mt19937 generator; //(42);
	all_sensors << d(generator);
	return all_sensors;
}


typedef MotionModel motion_model_type;
typedef ObservationModel obs_motion_type;
}

int main() {

    using namespace example_1;

    try{
		
		typedef motion_model_type::state_type state_type;
		obs_motion_type obs;
		DynMat<real_t> H(ObservationModel::VEC_SIZE, MotionModel::VEC_SIZE);
		H << 1.0;
		
		obs.add_matrix("H", H);
		
		// the motion model
		motion_model_type motion;
		
		// set the initial state
		DynVec<real_t> x_init(MotionModel::VEC_SIZE);
		x_init << 0;
		motion.set_state(x_init);
		
		//...and  the matrix describing the motion dynamics
		DynMat<real_t> F(MotionModel::VEC_SIZE, MotionModel::VEC_SIZE);
		F << 1.0;
		
		motion.add_matrix("F", F);
		
		KalmanFilterConfig<motion_model_type, obs_motion_type> kf_config;
		kf_config.motion_model = &motion;
		kf_config.observation_model = &obs;
		
		// the KalmanFilter to use
		KalmanFilter<motion_model_type, obs_motion_type> kf(kf_config);
		
		// set up the matrices
		DynMat<real_t> P(MotionModel::VEC_SIZE, MotionModel::VEC_SIZE);
		P << 1.0;
		
		DynMat<real_t> Q(MotionModel::VEC_SIZE, MotionModel::VEC_SIZE);
		Q << 1.0e-5;
			 
		DynMat<real_t> R(ObservationModel::VEC_SIZE, ObservationModel::VEC_SIZE);
		R << STD*STD;
		
		DynMat<real_t> B(MotionModel::VEC_SIZE, MotionModel::VEC_SIZE);
		B << 0.0;
		
		kf.with_matrix("P", P)
		  .with_matrix("Q", Q)
		  .with_matrix("R", R)
		  .with_matrix("B", B);
		
		
		// the input to the filter
		std::map<std::string, std::any > kf_input; 
		
		auto n_steps = static_cast<uint_t>(SIM_TIME / DT);
		
		BOOST_LOG_TRIVIAL(info)<<"Expected number of time steps: "<<n_steps;
		std::vector<state_type> rows;
		rows.reserve(n_steps);
		
		auto current_time = 0.0;
		while(current_time <= SIM_TIME){
			
			auto u = Cmd::cmd();
			auto w = DynVec<real_t>(u.size());
			w << 0.0;
			auto z = ObservationModel::sensors();
			
			kf_input["u"] = u;
			kf_input["w"] = w;
			kf_input["z"] = z;
			
			auto& state_vec = kf.estimate(kf_input);
			
			BOOST_LOG_TRIVIAL(info)<<"Time: "<<current_time<<" solution "<<state_vec[0];
			
			state_type row(1 + state_vec.size());
			row[0] = current_time;
			
			for(uint_t i =0; i < static_cast<uint_t>(state_vec.size()); ++i){
				row[i + 1] = state_vec[i];
			}
			
			rows.push_back(row);
			current_time += DT;
		}
		
		CSVWriter csv_writer("state.csv");
		csv_writer.open();
		
		for(auto& r:rows){
			csv_writer.write_row(r);
		}

    }
    catch(std::exception& e){
        std::cout<<e.what()<<std::endl;
    }
    catch(...){

        std::cout<<"Unknown exception occured"<<std::endl;
    }

   return 0;
}
