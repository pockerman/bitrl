#include "bitrl/bitrl_config.h"


#ifdef BITRL_WEBOTS

#include "rlenvs/rigid_bodies/webots_robots/epuck_robot.h"

#include <iostream>
#include <exception>

namespace rlenvscpp{
namespace rigid_bodies{
namespace webots_robots{
	
using namespace webots;

const std::string EpuckRobot::name = "epuck";

EpuckRobot::EpuckRobot()
:
robot_(),
position_sensors_(2, nullptr),
motors_(2, nullptr),
proximity_sensors_(EpuckRobot::DISTANCE_SENSORS_NUMBER, nullptr)
{
	robot_ = std::make_shared<webots::Supervisor>();
}


void 
EpuckRobot::reset(){
	
	// this seems to keep the odometry
	// always at zero!!!!
	//robot_ -> simulationReset(); 
	
	// this makes everything crash
	//robot_ -> getSelf() -> restartController();
	
	// I cannot create a new instance as the simulator
	// throws: Only one instance of the Robot class should be created
	//robot_ = std::make_shared<webots::Supervisor>();
	robot_ -> simulationResetPhysics();
	robot_ -> getSelf() -> resetPhysics();
	robot_ -> getFromDef("e-puck") -> resetPhysics();
	
	const real_t new_translation[3] = {0.0, 0.0, 0.0};
	robot_ -> getSelf() -> getField("translation") -> setSFVec3f(new_translation);
	robot_ -> getFromDef("e-puck") -> getField("translation")->setSFVec3f(new_translation);
	
	// x, y, z, angle
	const real_t new_rotation[4] = {0.0, 0.0, 1.0, 0.0};
	robot_ -> getSelf() -> getField("rotation") -> setSFRotation(new_rotation);
	robot_ -> getFromDef("e-puck") -> getField("rotation")->setSFRotation(new_rotation);
}


void 
EpuckRobot::activate_motor(uint_t motor_id, real_t init_velocity){
	
	if(motor_id == 0){
		motors_[0] = robot_ -> getMotor("left wheel motor");
		motors_[0] -> setPosition(INFINITY);
		motors_[0] -> setVelocity(init_velocity);
	}
	
	if(motor_id == 1){
		motors_[1] = robot_ -> getMotor("right wheel motor");
		motors_[1] -> setPosition(INFINITY);
		motors_[1] -> setVelocity(init_velocity);
	}
}

void 
EpuckRobot::activate_position_sensor(uint_t sensor_id, uint_t time_step){
	
	if(sensor_id == 0){
		position_sensors_[sensor_id] = robot_->getPositionSensor("left wheel sensor");
		position_sensors_[sensor_id] -> enable(time_step);
	}
	
	if(sensor_id == 1){
		position_sensors_[sensor_id] = robot_->getPositionSensor("right wheel sensor");
		position_sensors_[sensor_id] -> enable(time_step);
	}
	
}

void 
EpuckRobot::activate_proximity_sensor(const std::string& name, uint_t time_step){
	
	if(name == "ps0"){
		proximity_sensors_[0] = robot_ -> getDistanceSensor("ps0");
		proximity_sensors_[0] -> enable(time_step);
	}
	
	if(name == "ps1"){
		proximity_sensors_[1] = robot_ -> getDistanceSensor("ps1");
		proximity_sensors_[1] -> enable(time_step);
	}
	
	if(name == "ps2"){
		proximity_sensors_[2] = robot_ -> getDistanceSensor("ps2");
		proximity_sensors_[2] -> enable(time_step);
	}
	
	if(name == "ps3"){
		proximity_sensors_[3] = robot_ -> getDistanceSensor("ps3");
		proximity_sensors_[3] -> enable(time_step);
	}
	
	if(name == "ps4"){
		proximity_sensors_[4] = robot_ -> getDistanceSensor("ps4");
		proximity_sensors_[4] -> enable(time_step);
	}
	
	if(name == "ps5"){
		proximity_sensors_[5] = robot_ -> getDistanceSensor("ps5");
		proximity_sensors_[5] -> enable(time_step);
	}
	
	if(name == "ps6"){
		proximity_sensors_[6] = robot_ -> getDistanceSensor("ps6");
		proximity_sensors_[6] -> enable(time_step);
	}
	
	if(name == "ps7"){
		proximity_sensors_[7] = robot_ -> getDistanceSensor("ps7");
		proximity_sensors_[7] -> enable(time_step);
	}
}

real_t 
EpuckRobot::get_distance_value_from_sensor(const std::string& sensor_name)const{
	
	if(name == "ps0"){
		return proximity_sensors_[0] -> getValue();
	}
	
	if(name == "ps1"){
		return proximity_sensors_[1] -> getValue();
	}
	
	if(name == "ps2"){
		return proximity_sensors_[2] -> getValue();
	}
	
	if(name == "ps3"){
		return proximity_sensors_[3] -> getValue();
	}
	
	if(name == "ps4"){
		return proximity_sensors_[4] -> getValue();
	}
	
	if(name == "ps5"){
		return proximity_sensors_[5] -> getValue();
	}
	
	if(name == "ps6"){
		return proximity_sensors_[6] -> getValue();
	}
	
	if(name == "ps7"){
		return proximity_sensors_[7] -> getValue();
	}
	
	throw std::logic_error("Invalid sensor name: " + sensor_name);
}

std::vector<real_t> 
EpuckRobot::read_distance_sensors()const{
	
	std::vector<real_t> distances_;
	distances_.reserve(proximity_sensors_.size());
	
	for(uint_t s=0; s < proximity_sensors_.size(); ++s){
		
		if(proximity_sensors_[s] == nullptr){
			throw std::logic_error("Cannot read null sensor: " + std::to_string(s));
		}

		distances_.push_back(proximity_sensors_[s] -> getValue());
	}
	return distances_;
}


RBTranslation 
EpuckRobot::get_position()const{
	
	//const real_t *position = robot_ -> getFromDef("e-puck") -> getPosition(); //getSelf() -> getPosition(); 
	//getFromDef("e-puck") -> getPosition();
	const real_t *position = robot_ -> getSelf() -> getField("translation") -> getSFVec3f(); //getPosition(); 
	return {position[0], position[1], position[2]}; 
}

RBRotation 
EpuckRobot::get_rotation()const{
	// review this and fix this function
	// https://cyberbotics.com/doc/reference/supervisor?tab-language=c++#wb_supervisor_node_get_orientation
	const real_t *orientation = robot_ -> getSelf() -> getOrientation(); 
	//getgetFromDef("e-puck") -> getOrientation();
	return {orientation[0], orientation[1], orientation[2]}; 
}

EpuckOdometry 
EpuckRobot::compute_odometry()const{
	
	
  if(position_sensors_[0] == nullptr || position_sensors_[1] == nullptr){
	 throw std::logic_error("Cannot read null sensor"); 
  }
  
  real_t l = position_sensors_[0] -> getValue();
  real_t r = position_sensors_[1] -> getValue(); 
  real_t dl = l * WHEEL_RADIUS;         // distance covered by left wheel in meter
  real_t dr = r * WHEEL_RADIUS;         // distance covered by right wheel in meter
  real_t da = (dr - dl) / AXLE_LENGTH;  // delta orientation
  
  return EpuckOdometry(dl, dr, da);
	
}

std::ostream& 
EpuckOdometry::print(std::ostream& out)const noexcept{
	
	out<<"dl: "<< dl <<std::endl;
	out<<"dr: "<< dr <<std::endl;
	out<<"da: "<< da <<std::endl;
	return out;
}
	
}
}
}


#endif