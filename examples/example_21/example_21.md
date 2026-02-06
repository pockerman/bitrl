# EXample 1: Kalman filtering

\page bitrl_example_21 BitRL Example 21: Kalman filtering

In this example we will use the \ref bitrl::estimation::KalmanFilter "bitrl::estimation::KalmanFilter" class in order to estimate 
a scalar variable. The example is taken from the 
paper <a href="https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf">An Introduction to the Kalman Filter</a> by
Greg Welch and Gary Bishop. In fact the implementation of the \ref bitrl::estimation::KalmanFilter "bitrl::estimation::KalmanFilter" follows this paper.

@code{.cpp}
#include "cubeai/base/cubeai_types.h"
#include "cubeai/estimation/kalman_filter.h"
#include "cubeai/estimation/kf_model_base.h"
#include "cubeai/utils/iteration_counter.h"
#include "cubeai/io/csv_file_writer.h"

#include <boost/log/trivial.hpp>
#include <iostream>
#include <unordered_map>
#include <any>
#include <vector>
#include <random>

namespace example_1
{

using cubeai::real_t;
using cubeai::uint_t;
using cubeai::DynMat;
using cubeai::DynVec;
using cubeai::estimation::KalmanFilterConfig;
using cubeai::estimation::KalmanFilter;
using cubeai::estimation::KFMotionModelBase;
using cubeai::estimation::KFModelBase;
using cubeai::utils::IterationCounter;
using cubeai::io::CSVWriter;


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
@endcode

Running the code above produces the following output:

@code
[2024-12-01 15:05:58.695347] [0x00007f53a0e16000] [info]    Expected number of time steps: 50
[2024-12-01 15:05:58.695480] [0x00007f53a0e16000] [info]    Time: 0 solution -0.360215
[2024-12-01 15:05:58.695528] [0x00007f53a0e16000] [info]    Time: 1 solution -0.362008
[2024-12-01 15:05:58.695569] [0x00007f53a0e16000] [info]    Time: 2 solution -0.36261
[2024-12-01 15:05:58.695610] [0x00007f53a0e16000] [info]    Time: 3 solution -0.362912
[2024-12-01 15:05:58.695650] [0x00007f53a0e16000] [info]    Time: 4 solution -0.363094
[2024-12-01 15:05:58.695691] [0x00007f53a0e16000] [info]    Time: 5 solution -0.363215
[2024-12-01 15:05:58.695731] [0x00007f53a0e16000] [info]    Time: 6 solution -0.363302
[2024-12-01 15:05:58.695773] [0x00007f53a0e16000] [info]    Time: 7 solution -0.363368
[2024-12-01 15:05:58.695816] [0x00007f53a0e16000] [info]    Time: 8 solution -0.363419
[2024-12-01 15:05:58.695858] [0x00007f53a0e16000] [info]    Time: 9 solution -0.36346
[2024-12-01 15:05:58.695901] [0x00007f53a0e16000] [info]    Time: 10 solution -0.363493
[2024-12-01 15:05:58.695941] [0x00007f53a0e16000] [info]    Time: 11 solution -0.363521
[2024-12-01 15:05:58.695979] [0x00007f53a0e16000] [info]    Time: 12 solution -0.363545
[2024-12-01 15:05:58.696021] [0x00007f53a0e16000] [info]    Time: 13 solution -0.363566
[2024-12-01 15:05:58.696061] [0x00007f53a0e16000] [info]    Time: 14 solution -0.363583
[2024-12-01 15:05:58.696104] [0x00007f53a0e16000] [info]    Time: 15 solution -0.363599
[2024-12-01 15:05:58.696145] [0x00007f53a0e16000] [info]    Time: 16 solution -0.363613
[2024-12-01 15:05:58.696185] [0x00007f53a0e16000] [info]    Time: 17 solution -0.363626
[2024-12-01 15:05:58.696226] [0x00007f53a0e16000] [info]    Time: 18 solution -0.363637
[2024-12-01 15:05:58.696266] [0x00007f53a0e16000] [info]    Time: 19 solution -0.363647
[2024-12-01 15:05:58.696307] [0x00007f53a0e16000] [info]    Time: 20 solution -0.363656
[2024-12-01 15:05:58.696347] [0x00007f53a0e16000] [info]    Time: 21 solution -0.363664
[2024-12-01 15:05:58.696388] [0x00007f53a0e16000] [info]    Time: 22 solution -0.363672
[2024-12-01 15:05:58.696428] [0x00007f53a0e16000] [info]    Time: 23 solution -0.363679
[2024-12-01 15:05:58.696468] [0x00007f53a0e16000] [info]    Time: 24 solution -0.363686
[2024-12-01 15:05:58.696509] [0x00007f53a0e16000] [info]    Time: 25 solution -0.363692
[2024-12-01 15:05:58.696549] [0x00007f53a0e16000] [info]    Time: 26 solution -0.363697
[2024-12-01 15:05:58.696590] [0x00007f53a0e16000] [info]    Time: 27 solution -0.363703
[2024-12-01 15:05:58.696630] [0x00007f53a0e16000] [info]    Time: 28 solution -0.363708
[2024-12-01 15:05:58.696671] [0x00007f53a0e16000] [info]    Time: 29 solution -0.363712
[2024-12-01 15:05:58.696711] [0x00007f53a0e16000] [info]    Time: 30 solution -0.363717
[2024-12-01 15:05:58.696752] [0x00007f53a0e16000] [info]    Time: 31 solution -0.363721
[2024-12-01 15:05:58.696792] [0x00007f53a0e16000] [info]    Time: 32 solution -0.363725
[2024-12-01 15:05:58.696832] [0x00007f53a0e16000] [info]    Time: 33 solution -0.363728
[2024-12-01 15:05:58.696873] [0x00007f53a0e16000] [info]    Time: 34 solution -0.363732
[2024-12-01 15:05:58.696914] [0x00007f53a0e16000] [info]    Time: 35 solution -0.363735
[2024-12-01 15:05:58.696954] [0x00007f53a0e16000] [info]    Time: 36 solution -0.363738
[2024-12-01 15:05:58.696994] [0x00007f53a0e16000] [info]    Time: 37 solution -0.363741
[2024-12-01 15:05:58.697035] [0x00007f53a0e16000] [info]    Time: 38 solution -0.363744
[2024-12-01 15:05:58.697081] [0x00007f53a0e16000] [info]    Time: 39 solution -0.363746
[2024-12-01 15:05:58.697122] [0x00007f53a0e16000] [info]    Time: 40 solution -0.363749
[2024-12-01 15:05:58.697163] [0x00007f53a0e16000] [info]    Time: 41 solution -0.363751
[2024-12-01 15:05:58.697203] [0x00007f53a0e16000] [info]    Time: 42 solution -0.363754
[2024-12-01 15:05:58.697244] [0x00007f53a0e16000] [info]    Time: 43 solution -0.363756
[2024-12-01 15:05:58.697285] [0x00007f53a0e16000] [info]    Time: 44 solution -0.363758
[2024-12-01 15:05:58.697325] [0x00007f53a0e16000] [info]    Time: 45 solution -0.36376
[2024-12-01 15:05:58.697366] [0x00007f53a0e16000] [info]    Time: 46 solution -0.363762
[2024-12-01 15:05:58.697407] [0x00007f53a0e16000] [info]    Time: 47 solution -0.363764
[2024-12-01 15:05:58.697447] [0x00007f53a0e16000] [info]    Time: 48 solution -0.363766
[2024-12-01 15:05:58.697488] [0x00007f53a0e16000] [info]    Time: 49 solution -0.363768
[2024-12-01 15:05:58.697528] [0x00007f53a0e16000] [info]    Time: 50 solution -0.363769
@endcode

Feel free to experiment with ```MU``` and ```STD``` to see how the filter performs.

