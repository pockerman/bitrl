Working with Webots
====================

In this section we will discuss how to work with webots-based environments.
The instructions here assume that you work with a Linux OS.


Working with webots-based environments requires that you install the 
Webots simulator. General instructions how to achieve this you 
can find at `Installing Webots <https://cyberbotics.com/doc/guide/installing-webots>`_.
In addition, you can also install Webots from source: 
`Installation of the Webots Development Environment <https://github.com/cyberbotics/webots/wiki>`_.
For Linux boxes instructions can be found at `Linux installation <https://github.com/cyberbotics/webots/wiki/Linux-installation/>`_.


Assuming that you have Webots installed on your machine, you need to install ``rlenvscpp`` with
``ENABLE_WEBOTS`` set to on when configuring the library i.e.

.. code-block::

	mkdir build 
	cd build 
	cmake -DENABLE_WEBOTS=ON ..
	make install
	
You will need to edit the main CMakeLists.txt file and specify the paths for the 
variables ``WEBOTS_INCLUDE_DIRS`` and ``WEBOTS_LIB_DIRS`` to match your environment.
Notice that ``cmake`` will not look into any default locations for this variables
so you need to fill these in even if you do a system wide installation.

.. code-block::

	...
	
	IF(ENABLE_WEBOTS)
		# we have webots include the directories
		SET(WEBOTS_LIB_DIRS "set/your/path")    
		SET(WEBOTS_INCLUDE_DIRS "set/your/path")
	
		INCLUDE_DIRECTORIES(${WEBOTS_INCLUDE_DIRS})
		LINK_DIRECTORIES(${WEBOTS_LIB_DIRS})
		SET(RLENVSCPP_WEBOTS ON)
	ELSE()
		MESSAGE( WARNING  "Building without Webots.")
	ENDIF()
	
	...

Once the ``rlenvscpp`` library is built, edit your ``~/.bashrc`` and amend the variable ``LD_LIBRARY_PATH`` as
follows:

.. code-block::
	
	export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/your/path/to/rlenvscpplib"
	
We need this in order for the Webots simulator to be able to find ``rlenvscpplib``.
Source the updated ````~/.bashrc`` file: 

.. code-block::

	source ~/.bashrc
	
or open a new terminal.

You are now ready to develop and use environments for your application.
Follow the Webots tutorials in order to do so: `Tutorial 1: Your First Simulation in Webots (30 Minutes) <https://www.cyberbotics.com/doc/guide/tutorial-1-your-first-simulation-in-webots?tab-language=c>`_
Once your environment is ready, edit the associated Makefile and amend the following lines

.. code-block::

	...
	
	CFLAGS = -std=c++20 -Wno-unused-result
	INCLUDE = -I"/your/path/to/rlenvs_from_cpp/src"
	LIBRARIES = -L"/your/path/to/rlenvscpplib/dir" -lrlenvscpplib
	
	...
	
Update the controller source file according to your needs. Here is an example taken
from ``rlenvscpp`` examples:

.. code-block::

	#include "rlenvs/rlenvscpp_config.h"

	#ifdef RLENVSCPP_WEBOTS

	#include "rlenvs/rlenvs_types_v2.h"
	#include "rlenvs/envs/time_step.h"
	#include "rlenvs/envs/webots_envs/epuck_simple_grid_world.h"
	
	#include <iostream>
	#include <memory>
	#include <vector>
	#include <unordered_map>
	#include <any>
	#include <exception>
	#include <chrono>
	#include <thread>
	


	// All the webots classes are defined in the "webots" namespace
	using namespace webots;
	using namespace rlenvscpp;
	
	namespace webots_example_1{
			
	using rlenvscpp::envs::webots_envs::EpuckSimpleGridWorld;
	using rlenvscpp::uint_t;
	
	}

	// This is the main program of your controller.
	// It creates an instance of your Robot instance, launches its
	// function(s) and destroys it at the end of the execution.
	// Note that only one instance of Robot should be created in
	// a controller program.
	// The arguments of the main function can be specified by the
	// "controllerArgs" field of the Robot node
	int main() {
	
	
		using namespace webots_example_1;
   
	try
	
			// create the environment
			EpuckSimpleGridWorld env;
		
			// create the environment...this initializes
			// the robot sensors and motors
			std::unordered_map<std::string, std::any> options;
			options["right_motor_init_velocity"] = 1.25; // 2.0;
			options["left_motor_init_velocity"] = 1.25; // 2.0;
			options["sim_time_step"] = std::any(static_cast<uint_t>(64));
			
			// above this reading we assume there is a wall
			options["max_dist_sensor_reading_goal"] = 120.0;
			
			// we don't want the robot to be far away from the
			// wall either
			options["min_dist_sensor_reading_goal"] = 80.0;
	
			env.make("v0", options); 
	
			// reset the environment...this will reload the 
			// whole simulation world
			env.reset();
			
			// access the robot
			auto& robot = env.get_robot();
			
			auto time_step = robot.get_basic_time_step();
			std::cout<<"Basic time step used: "<<time_step<<std::endl;
			
			real_t total_reward = 0.0;
			
			// do 500 episodes 270
			for(uint_t e=0; e<272; ++e){
				
				std::cout<<"At step: "<<e<<std::endl;
				std::cout<<"Robot left motor velocity: "<<robot.get_motor_velocity(0)<<std::endl;
				std::cout<<"Robot right motor velocity: "<<robot.get_motor_velocity(1)<<std::endl;
				
				auto r_position = robot.get_position();
			
				std::cout<<"Robot position: "<<r_position<<std::endl;
			
				// what is the max reward
				auto reward = env.compute_reward();
				std::cout<<"Reward predicted: "<<reward<<std::endl;
				
				// move forward unless the
				// reward is the highest
				auto action = 1;
				if(reward == 10.0){
					action = 0;
				}
				
				std::cout<<"Executing action: "<<action<<std::endl;
				
				// step in the environment
				auto time_step = env.step(action);
					
				total_reward += time_step.reward();
				
				std::cout<<"Total reward at episode: "<<e<<" "<<total_reward<<std::endl;
				
				auto distances = robot.read_distance_sensors();
				for(uint_t s=0; s < distances.size(); ++s){
						std::cout<<"Sensor: "<<s<<" distance: " << distances[s]<<std::endl;
				}
				
				if(time_step.done()){
					
					std::cout<<"Reset the environment..."<<std::endl;
					env.reset();
					break;
				}
			}
	
			std::cout<<"Total reward "<<total_reward<<std::endl;
			
			// this will quit the simulation window
			//env.close();
			std::this_thread::sleep_for(std::chrono::milliseconds(2));
			env.pause_simulation();
		}
	   catch(const std::logic_error& e){
		   std::cout<<e.what()<<std::endl;
	   }

			return 0;
	}
	#else
		#include <iostream>
		int main(){

			std::cout<<"This example requires Webots enabled. Rebuild the library with -DENABLE_WEBOTS=ON"<<std::endl;
			return 0;
		}
	#endif
	

Note that you will need to compile your controller via the Webots UI. See the tutorial above.