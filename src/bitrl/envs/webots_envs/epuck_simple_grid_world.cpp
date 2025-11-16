#include "bitrl/bitrl_config.h"

#ifdef BITRL_WEBOTS

#include "rlenvs/envs/webots_envs/epuck_simple_grid_world.h"
#include "rlenvs/envs/time_step.h"
#include "rlenvs/envs/time_step_type.h"
#include "rlenvs/utils/maths/math_utils.h"
#include "rlenvs/rigid_bodies/body_translation.h"


#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Motor.hpp>

#include <cmath>
#include <exception>

namespace rlenvscpp{
namespace envs{
namespace webots_envs{
	
	
const std::string EpuckSimpleGridWorld::name = "EpuckSimpleGridWorld";
	
EpuckSimpleGridWorld::EpuckSimpleGridWorld()
:
WebotsEnvBase<TimeStep<state_type>,
			  ContinuousVectorStateDiscreteActionEnv<
										 3, // the state space has size 3 
										 1, // the action space ends at 1
										 0, // the action space starts at 0
										 real_t
										>
			>(static_cast<uint_t>(0), "EpuckSimpleGridWorld"),
			robot_()
{}



void 
EpuckSimpleGridWorld::make(const std::string& version,
                           const std::unordered_map<std::string, std::any>& options){
							   
	if(this -> is_created()){
        return;
    }
	
	build_make_options_(options);
	activate_robot_();
	
	// set the version and set the board
    // to created
	this->set_version_(version);
    this->make_created_();
}

EpuckSimpleGridWorld::time_step_type 
EpuckSimpleGridWorld::reset(uint_t /*seed*/,
						    const std::unordered_map<std::string, std::any>& /*options*/){
	

	// reset the robot this will reset the whole
	// simulation environment
	robot_.reset(); 
	
	// re-activate the robot?
	activate_robot_();
	
	rigid_bodies::RBTranslation position = robot_.get_position();
	std::vector<real_t> state{position.x, position.y, position.z};
	return EpuckSimpleGridWorld::time_step_type(TimeStepTp::FIRST, 
	                                            0.0, 
												state);
								
}


void 
EpuckSimpleGridWorld::build_make_options_(const std::unordered_map<std::string, std::any>& options){
	
	
	std::unordered_map<std::string, std::any> options_;
	auto sim_time_step_itr = options.find("sim_time_step");
	
	if(sim_time_step_itr != options.end()){
		options_["sim_time_step"] = sim_time_step_itr->second;
	}
	else{
		
		options_["sim_time_step"] = this -> DEFAULT_SIM_TIME_STEP;
	}
	
	auto max_distance_goal_itr = options.find("max_dist_sensor_reading_goal");
	if(max_distance_goal_itr != options.end()){
		options_["max_dist_sensor_reading_goal"] = max_distance_goal_itr->second;
	}
	else{
		throw std::logic_error("option: max_dist_sensor_reading_goal is missing");
	}
	
	auto min_distance_goal_itr = options.find("min_dist_sensor_reading_goal");
	if(min_distance_goal_itr != options.end()){
		options_["min_dist_sensor_reading_goal"] = min_distance_goal_itr->second;
	}
	else{
		throw std::logic_error("option: min_dist_sensor_reading_goal is missing");
	}
	
	auto left_pos_sensor_time_step_itr = options.find("left_pos_sensor_time_step");
	if(left_pos_sensor_time_step_itr != options.end()){
		options_["left_pos_sensor_time_step"] = sim_time_step_itr->second;
	}
	else{
		options_["left_pos_sensor_time_step"] = this -> DEFAULT_SIM_TIME_STEP;
	}
	
	auto right_pos_sensor_time_step_itr = options.find("right_pos_sensor_time_step");
	if(right_pos_sensor_time_step_itr != options.end()){
		options_["right_pos_sensor_time_step"] = sim_time_step_itr->second;
	}
	else{
		options_["right_pos_sensor_time_step"] = this -> DEFAULT_SIM_TIME_STEP;
	}
	
	auto left_motor_init_velocity_itr = options.find("left_motor_init_velocity");
	if(left_motor_init_velocity_itr != options.end()){
		options_["left_motor_init_velocity"] = left_motor_init_velocity_itr->second;
	}
	else{
		options_["left_motor_init_velocity"] = std::any(static_cast<real_t>(0.0));
	}
	
	auto right_motor_init_velocity_itr = options.find("right_motor_init_velocity");
	if(right_motor_init_velocity_itr != options.end()){
		options_["right_motor_init_velocity"] = right_motor_init_velocity_itr->second;
	}
	else{
		options_["right_motor_init_velocity"] = std::any(static_cast<real_t>(0.0));
	}
	
	auto ps_itr = options.find("distance_sensor_time_step");
	if(ps_itr != options.end()){
		options_["distance_sensor_time_step"] = ps_itr->second;
	}
	else{
		options_["distance_sensor_time_step"] = this -> DEFAULT_SIM_TIME_STEP;
	}
	
	this->set_make_options_(options_);
	
}

void 
EpuckSimpleGridWorld::activate_robot_(){
	
	auto& options_ = this -> make_options();
	auto left_pos_sensor_time_step = this -> template read_option<uint_t>("left_pos_sensor_time_step");
	robot_.activate_position_sensor(0, std::any_cast<uint_t>(left_pos_sensor_time_step));
	
	auto right_pos_sensor_time_step = this -> template read_option<uint_t>("right_pos_sensor_time_step"); 
	robot_.activate_position_sensor(1, std::any_cast<uint_t>(right_pos_sensor_time_step));
	
	
	auto left_motor_init_velocity = this -> template read_option<real_t>("left_motor_init_velocity"); 
	robot_.activate_motor(0, std::any_cast<real_t>(left_motor_init_velocity));
	
	auto right_motor_init_velocity = this -> template read_option<real_t>("right_motor_init_velocity"); 
	robot_.activate_motor(1, std::any_cast<real_t>(right_motor_init_velocity));
	
	auto distance_sensor_time_step = this -> template read_option<uint_t>("distance_sensor_time_step");
	for(uint_t i=0; i<8; ++i){
		std::string name_s = "ps"+std::to_string(i);
		robot_.activate_proximity_sensor(name_s, distance_sensor_time_step); 
	}
}


void 
EpuckSimpleGridWorld::close(){
	
	// close this world
	robot_.stop_simulation();
	
	this -> base_type::close();
}

void 
EpuckSimpleGridWorld::pause_simulation(){
	robot_.pause_simulation();
}

real_t 
EpuckSimpleGridWorld::compute_reward()const{
	
	std::vector<real_t> dist_sensor_readings = robot_.read_distance_sensors();
	
	// check first for a collision
	// if there is a sensor reading above a threshold
	// i.e. max_dist_sensor_reading_goal then there is
	// a collision
	auto max_reading = utils::maths::max(dist_sensor_readings);
	auto max_dist_sensor_reading_goal = read_option<real_t>("max_dist_sensor_reading_goal");
	if(max_reading - max_dist_sensor_reading_goal > 1.0e-4){
		
		// this means the robot crashed
		return  -10.0;
	}
	else{
		
		// there is no reading that is above 
		// the max threshold
		
		auto min_reading = utils::maths::min(dist_sensor_readings);
		auto min_dist_sensor_reading_goal = read_option<real_t>("min_dist_sensor_reading_goal");
		
		if(min_dist_sensor_reading_goal > min_reading && min_dist_sensor_reading_goal < max_reading){
			return 10;
		}

		// there is a minimum reading above the min reading
		// so this means the robot is somewhere where [min_reading, max_reading]
//		if(min_reading - min_dist_sensor_reading_goal > 1.0e-4){
//			return 10.0;
//		}
//		else{
			// is a win
		return 1.0;
		//}
	}
}


EpuckSimpleGridWorld::time_step_type
EpuckSimpleGridWorld::on_stop_(){
	
	auto r_position = robot_.get_position();
	auto reward = compute_reward();
	std::vector<real_t> state{r_position.x, r_position.y, r_position.z};
	return EpuckSimpleGridWorld::time_step_type(TimeStepTp::LAST, reward, state);
}

EpuckSimpleGridWorld::time_step_type			
EpuckSimpleGridWorld::on_move_(){
	
	auto sim_time_step = read_option<uint_t>("sim_time_step");
	
	
	
	/// the robot was instructed to
	/// move
	auto step_idx = robot_.step(sim_time_step);
	
	// now check the reward
	auto reward = compute_reward();
	
	auto r_position = robot_.get_position();
	
	if(reward == -1.0){
		
		std::vector<real_t> state{r_position.x, r_position.y, r_position.z};
		return EpuckSimpleGridWorld::time_step_type(TimeStepTp::MID, -1.0*reward, state);
	}
	else{
		// reward == -10.0 means that the robot crashed
		// so for the current robot velocity this wasn't the best
		// choice
		if(reward == -10.0){
			std::vector<real_t> state{r_position.x, r_position.y, r_position.z};
			return EpuckSimpleGridWorld::time_step_type(TimeStepTp::LAST, reward, state);
		}
		else{
			// this is reward == 10 this is winning
			// but the robot has to actually call stop
			// so we just give reward 1.0
			std::vector<real_t> state{r_position.x, r_position.y, r_position.z};
			return EpuckSimpleGridWorld::time_step_type(TimeStepTp::MID, 1.0, state);
		}
	}
}

EpuckSimpleGridWorld::time_step_type 
EpuckSimpleGridWorld::step(const action_type& action){
	
	if( (action != 0) && (action != 1)){
		throw std::logic_error("action should be either 0 or 1");
	}
	
	if(action == 0){
		
		// the robot was instructed to
		// stop.
		this -> get_current_time_step_ () = on_stop_();
		return this -> get_current_time_step_ ();
	}
	
	this -> get_current_time_step_ () = on_move_();
	return this -> get_current_time_step_ ();
	
	
}


}
}
}

#endif 

