How to use 
=============

The following is an example how to use the 
``FrozenLake``   environment from <a href="https://github.com/Farama-Foundation/Gymnasium/tree/main">Gymnasium</a>.

.. code-block::

	#include "rlenvs/rlenvs_types_v2.h"
	#include "rlenvs/envs/gymnasium/toy_text/frozen_lake_env.h"
	#include "rlenvs/envs/api_server/apiserver.h"

	#include <iostream>
	#include <string>
	#include <unordered_map>
	#include <any>

	namespace example_1{

	const std::string SERVER_URL = "http://0.0.0.0:8001/api";

	using rlenvscpp::envs::gymnasium::FrozenLake;
	using rlenvscpp::envs::RESTApiServerWrapper;


	void test_frozen_lake(const RESTApiServerWrapper& server){

		FrozenLake<4> env(server);

		std::cout<<"Environame URL: "<<env.get_url()<<std::endl;

		// make the environment
		std::unordered_map<std::string, std::any> options;
		options.insert({"is_slippery", false});
		env.make("v1", options);

		std::cout<<"Is environment created? "<<env.is_created()<<std::endl;
		std::cout<<"Is environment alive? "<<env.is_alive()<<std::endl;
		std::cout<<"Number of valid actions? "<<env.n_actions()<<std::endl;
		std::cout<<"Number of states? "<<env.n_states()<<std::endl;

		// reset the environment
		auto time_step = env.reset(42, std::unordered_map<std::string, std::any>());

		std::cout<<"Reward on reset: "<<time_step.reward()<<std::endl;
		std::cout<<"Observation on reset: "<<time_step.observation()<<std::endl;
		std::cout<<"Is terminal state: "<<time_step.done()<<std::endl;

		//...print the time_step
		std::cout<<time_step<<std::endl;

		// take an action in the environment
		// 2 = RIGHT
		auto new_time_step = env.step(2);

		std::cout<<new_time_step<<std::endl;

		// get the dynamics of the environment for the given state and action
		auto state = 0;
		auto action = 1;
		auto dynamics = env.p(state, action);

		std::cout<<"Dynamics for state="<<state<<" and action="<<action<<std::endl;

		for(auto item:dynamics){

			std::cout<<std::get<0>(item)<<std::endl;
			std::cout<<std::get<1>(item)<<std::endl;
			std::cout<<std::get<2>(item)<<std::endl;
			std::cout<<std::get<3>(item)<<std::endl;
		}
		
		action = env.sample_action();
		new_time_step = env.step(action);

		std::cout<<new_time_step<<std::endl;
		
		// synchronize the environment
		env.sync(std::unordered_map<std::string, std::any>());
		
		auto copy_env = env.make_copy(1);
		copy_env.reset();
		
		std::cout<<"Org env cidx: "<<env.cidx()<<std::endl;
		std::cout<<"Copy env cidx: "<<copy_env.cidx()<<std::endl;
		
		copy_env.close();

		// close the environment
		env.close();

	}

	}


	int main(){

		using namespace example_1;
		
		RESTApiServerWrapper server(SERVER_URL, true);

		std::cout<<"Testing FrozenLake..."<<std::endl;
		example_1::test_frozen_lake(server);
		std::cout<<"===================="<<std::endl;
		return 0;
	}




In general, the environments exposed by the library  
follow the semantics in `Environment API and Semantics <https://github.com/deepmind/dm_env/blob/master/docs/index.md>`_ specification.
For more details see the <a href="doc/env_spec.md">```rlenvscpp``` environment specification</a> document.

The general use case is to build the library and link it with your driver code to access its functionality.
The environments specified as using REST in the tables above, that is all ``Gymnasium``, ``gym_pybullet_drones`` and ``GymWalk`` 
environments are accessed via a client/server pattern. Namely, they are exposed via an API developed using 
`FastAPI <https://fastapi.tiangolo.com/>`_.
You need to fire up the FastAPI server, see the dependencies, before using the environments in your code. 
To do so

.. code-block::

   ./start_uvicorn.sh


By default the ``uvicorn`` server listents on port 8001. Change this if needed. You can access the OpenAPI specification at

.. code-block::

	http://0.0.0.0:8001/docs


Note that currently the implementation is not thread/process safe i.e. if multiple threads/processes access the environment
a global instance of the environment is manipulated. Thus no session based environment exists.
However, you can create copies of the same environment and access this via its dedicate index.
If just one thread/process touches this specific environment you should be ok.
Notice that the FastAPI server only uses a single process to manage all the environments.
In addition, if  you need multiple instances of the same environment you can also  use one 
of the exissting vectorised environments (see table above).

Finally,   you can choose to launch several instances of ``uvirocrn`` (listening on different ports). 
However in this case you need to implement all the interactions logic yourself as currently no implementation exists to handle such a scenario.
