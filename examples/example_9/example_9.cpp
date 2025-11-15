#include "bitrl/bitrl_types_v2.h"
#include "bitrl/dynamics/diff_drive_dynamics.h"

#include <iostream>
#include <string>
#include <map>
#include <any>
#include <array>


int main(){

	using namespace bitrl::dynamics;
	using bitrl::uint_t;
	using bitrl::real_t;
	
	
	// don't update the matrices describing this
	// dynamics as we don't need it
	DiffDriveDynamics dd_dyn(DiffDriveDynamics::DynamicVersion::V1, false);
	
	
	// get the state
	auto& state = dd_dyn.get_state();
	
	std::cout<<"Initial state: \n";
	std::cout<<state<<std::endl;
	
	auto dt = 0.5;
	
	// set the time step for integration
	dd_dyn.set_time_step(dt);
	
	std::map<std::string, std::any> input;
	input["v"] = std::any(static_cast<real_t>(0.05)); //m/s
	input["w"] = std::any(static_cast<real_t>(0.0)); // rad/s
	input["errors"] = std::any(std::array<real_t, 2>{0.0}); // we may have some sensor errors
	
	// integrate one step
	dd_dyn.integrate(input);
	
	// state after one time step...
	// rigid body should move forward
	std::cout<<state<<std::endl;
	
    return 0;
}
