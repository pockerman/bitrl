Using ``rlenvscpp`` with Ray
=============================

Example <ray_example_1.html>`_ discussed how to use Ray for distributed computing.
In this example we will see how to use ``rlenvscpp`` alongside Ray.
Specifically, we will create a Ray `Task <https://docs.ray.io/en/latest/ray-core/tasks.html>`_
that sends action requests to an environment using a dummy policy, collects the time steps
and sends these to the application. This is an approach we can use when working 
with distributed reinforcement learning.

.. note::

	Currently, the way we include  ``rlenvscpp`` into a Ray based application
	is to build the library by specifying the source files. This is not
	optimal as we would like to precomplile the library and simply link it
	to the application. This however seems to be rather cryptinc with
	Bazel. If you know how to do this please let us know.

The driver code is shown below. Note that you need to have the Gymnasium REST API 
running for this example to work.


.. code-block::

	#include "rlenvs/rlenvs_types_v2.h"
	#include "rlenvs/envs/gymnasium/toy_text/frozen_lake_env.h"
	#include "rlenvs/envs/gymnasium/toy_text/taxi_env.h"
	#include "rlenvs/envs/gymnasium/toy_text/black_jack_env.h"
	#include "rlenvs/envs/gymnasium/toy_text/cliff_world_env.h"
	#include "rlenvs/envs/api_server/apiserver.h"
	#include "rlenvs/envs/time_step.h"
	#include "rlenvs/envs/time_step_type.h"
	
	// we need to include the Ray API
	#include <ray/api.h>
	#include <msgpack.hpp>
	
	#include <vector>
	#include <iostream>
	
	namespace example_2{
	
	const std::string SERVER_URL = "http://0.0.0.0:8001/api";
	
	using rlenvscpp::real_t;
	using rlenvscpp::uint_t;
	using rlenvscpp::envs::gymnasium::FrozenLake;
	using rlenvscpp::envs::gymnasium::Taxi;
	using rlenvscpp::envs::gymnasium::BlackJack;
	using rlenvscpp::envs::gymnasium::CliffWorld;
	using rlenvscpp::envs::RESTApiServerWrapper;
	using rlenvscpp::TimeStepTp;
	
	typedef FrozenLake<4>::time_step_type time_step_type;
	
	// Wrapper to the time step object
	// returned by FrozenLake so that it is compatible with
	// Ray
	struct SerialTimeStep
	{
		///
		/// \brief type_
		///
		uint_t type;
	
		///
		/// \brief reward_
		///
		real_t reward;
	
		///
		/// \brief obs_
		///
		uint_t obs;
	
		///
		/// \brief discount_. The discount_factor
		///
		real_t discount;
		
		MSGPACK_DEFINE(type, reward, obs, discount);
	};
	
	
	// This is the C++ function we want to execute
	std::vector<SerialTimeStep> 
	run_frozen_lake(){
	
		RESTApiServerWrapper server(SERVER_URL, true);
		
		FrozenLake<4> env(server);
	
		// make the environment
		std::unordered_map<std::string, std::any> options;
		options.insert({"is_slippery", false});
		env.make("v1", options);
	
		// reset the environment
		auto time_step = env.reset(42, std::unordered_map<std::string, std::any>());
	
		std::vector<SerialTimeStep> trajectory(10);
		
		auto action = 2;
		for(uint_t i=0; i < trajectory.size(); ++i){
			
			//auto action = env.sample_action();
			auto new_time_step = env.step(action);
			
			if(action == 2){
				action = 1;
			}
			else if(action == 1){
				action = 3;
			}
			else if (action == 3){
				action = 0;
			}
			else{
				action = 2;
			}
			
			SerialTimeStep step;
			step.reward = new_time_step.reward();
			step.discount = new_time_step.discount();
			step.obs = new_time_step.observation();
			step.type = static_cast<uint_t>(new_time_step.type());
			
			trajectory[i] = step;
		}
	
		// close the environment
		env.close();
		return trajectory;
	
	}
	}

	using namespace example_2;

	// In order to be able to execute this function
	// we need to register it using `RAY_REMOTE`.
	RAY_REMOTE(run_frozen_lake);

	int main(int argc, char **argv) {
		try{
			// We need to initialize Ray before using it
			ray::Init();
			
			// Invoke the above method as a Ray task.
			// This will immediately return an object ref (a future) and then create
			// a task that will be executed on a worker process.
			//auto res = ray::Task(run_frozen_lake).Remote();
			auto res = ray::Task(run_frozen_lake).Remote();
			
			// The result can be retrieved with ``ray::ObjectRef::Get``.
			auto trajectory = *res.Get();
			std::cout<<"Trajectory size is: "<<trajectory.size()<<std::endl;
			ray::Shutdown();
		}
		catch(...){
			ray::Shutdown();
		}
		return 0;
	}

	

Build and run the code
-----------------------

In order to build the code, we need to specify the sources for the ``rlenvscpp`` library.
The Bazel build file is shown below

.. code-block::

	cc_library(
		name = "ray_api",
		srcs = [
			"thirdparty/lib/libray_api.so",
		],
		hdrs = glob([
			"thirdparty/include/**/*.h",
			"thirdparty/include/**/*.hpp",
		]),
		linkopts = ["-Wl,-rpath,./"],
		strip_include_prefix = "thirdparty/include",
		visibility = ["//visibility:public"],
	)

	cc_library(
		name = "rlenvscpplib",
		srcs = glob([
					"thirdparty/include/rlenvs/*.cpp",
					"thirdparty/include/rlenvs/envs/*.cpp",
					"thirdparty/include/rlenvs/envs/api_server/*.cpp",
					"thirdparty/include/rlenvs/envs/gymnasium/*.cpp",
					"thirdparty/include/rlenvs/envs/gymnasium/toy_text/*.cpp",
					"thirdparty/include/rlenvs/envs/gymnasium/classic_control/*.cpp",
					"thirdparty/include/rlenvs/envs/gymnasium/classic_control/vector/*.cpp",
					"thirdparty/include/rlenvs/envs/gdrl/*.cpp",
					"thirdparty/include/rlenvs/envs/multi_armed_bandits/*.cpp",
					"thirdparty/include/rrlenvs/envs/gym_pybullet_drones/*.cpp",
					"thirdparty/include/rlenvs/envs/grid_world/*.cpp",
					"thirdparty/include/rlenvs/envs/connect2/*.cpp",
					"thirdparty/include/rlenvs/envs/webots_envs/*.cpp",
					"thirdparty/include/rlenvs/rigid_bodies/*.cpp",
					"thirdparty/include/rlenvs/rigid_bodies/webots_robots/*.cpp",
					"thirdparty/include/rlenvs/dynamics/*.cpp",
					"thirdparty/include/rlenvs/utils/*.cpp",
					"thirdparty/include/rlenvs/utils/io/*.cpp",
					"thirdparty/include/rlenvs/utils/io/tensor_board_server/*.cpp",
					"thirdparty/include/rlenvs/utils/maths/statistics/distributions/*.cpp",
					"thirdparty/include/rlenvs/utils/geometry/shapes/*.cpp",
					"thirdparty/include/rlenvs/utils/geometry/mesh/*.cpp",
					"thirdparty/include/rlenvs/utils/trajectory/*.cpp",
		]),
		hdrs = glob([
			"thirdparty/include/**/*.h",
			"thirdparty/include/**/*.hpp",
		]),
		copts = ["-L./thirdparty/lib"],
		linkopts = ["-Wl,-rpath=./thirdparty/lib/librlenvscpplib.so"],
		strip_include_prefix = "thirdparty/include",
		visibility = ["//visibility:public"],
	)


	cc_binary(
		name = "ray_example_2",
		srcs = ["ray_example_2.cpp"],
		data = ["ray_example_2.so",],
		linkstatic = True,
		deps = [
			":ray_api",
			":rlenvscpplib"
		],
	)

	cc_binary(
		name = "ray_example_2.so",
		srcs = glob([
			"ray_example_2.cpp",
		]),
		linkopts = ["-shared"],
		linkstatic = True,
		deps = [
			":ray_api",
			":rlenvscpplib"
		],
	)




