\page bitrl_example_19 bitrl Example 19: REINFORCE algorithm on CartPole

The DQN algorithm we used in examples <a href="../rl_example_12/rl_example_12.md">Example 12: DQN algorithm on Gridworld</a>
and <a href="../rl_example_15/rl_example_15.md">Example 15: DQN algorithm on Gridworld with experience replay</a> approximate a value function
and in particular the state-action value function i.e. $Q(s, \alpha)$. 
However, in reinforcement learning we are more interested in policies rather than value functions
This is because a policy dictates how  an agent behaves in a given state. Indeed one way to 
derive a policy from a state-action value function is to use a greedy approach i.e.

$$\pi(s) = max_{\alpha} Q(s, \alpha)$$

In this example, we will implement a policy-based method. In particular, we will use one of the earliest policy-based methods namely the
REINFORCE algorithm. Policy-based methods, as their name probably suggests, approximate policies directly. 
In particular, we will use a neural network to represent the policy $\pi$ and use variants of the gradient
descent algorithm in order to estimate the weights of the network.

The REINFORCE algorithm samples trajectories from the environment 
using the policy on hand and estimate the gradient using these samples. 
The true gradient can be then estimated using:


$$ \nabla J(\boldsymbol{\theta}_{t}) = E_{\pi} \left[ G_t \nabla log \pi(\alpha | S_t,  \boldsymbol{\theta}) \right] $$


where $G_t$ is the discounted cumulative reward at time step $t$. 
After the gradient calculation, we update the policy parameters using the following formula

$$\boldsymbol{\theta} = \boldsymbol{\theta} + \eta \nabla_{\boldsymbol{\theta}} J( \pi_{\boldsymbol{\theta}})$$

or by substituting the  expected value of the gradient:

$$\boldsymbol{\theta} = \boldsymbol{\theta} + \eta G_t\nabla_{\boldsymbol{\theta}} log \pi(\alpha | S_t,  \boldsymbol{\theta})$$

In this example we will use the ```gymnasium.CartPole``` environment.
Specifically, we will use the class <a href="https://github.com/pockerman/rlenvs_from_cpp/blob/master/src/rlenvs/envs/gymnasium/classic_control/cart_pole_env.h">CartPole</a> class from
<a href="https://github.com/pockerman/rlenvs_from_cpp/tree/master">rlenvs_from_cpp</a> library. This is a simple class that allows us to
sent HTTP requests to a server that actually runs the environment. We need this as ```gymnasium.CartPole``` is written in Python and we 
want to avoid the complexity of directly executing Python code from the C++ driver.

We will use the policy network from PyTorch examples: <a href="https://github.com/pytorch/examples/blob/main/reinforcement_learning/reinforce.py">reinforce.py</a>.
However, feel free to experiment with this. Note that the PyTorch implementation uses a baseline in the implementation.
We will use this approach in <a href="https://github.com/pockerman/bitrl/blob/master/examples/rl/rl_example_20/rl_example_20.md">REINFORCE with baseline algorithm on CartPole</a>


The REINFORCE algorithm is implemented in the class <a href="https://github.com/pockerman/bitrl/blob/master/include/cubeai/rl/algorithms/pg/simple_reinforce.h">ReinforceSolver</a>
The solver is passed to the ```RLSerialAgentTrainer``` class that manages the loop over the specified number of episodes.
The ```ReinforceSolver``` is derived from the ```RLSolverBase``` class and overrides some virtual methods.
The class ```ReinforceSolver``` accepts two template parameters:

- The environment type 
- The policy type 


The environment type is a standard argument for all RL solvers in ```bitrl```. The policy type represents the 
PyTorch model that we will use. Specifically, it should expose an ```act``` function that has the following signature

@code{.cpp}
template<typename StateTp>
std::tuple<uint_t, torch_tensor_t> act(const StateTp& state)
@endcode


## The _PolicyNetImpl_ class 

Below is the code for the network that implements the policy we want to use.
The network specification is taken from <a href="https://github.com/pytorch/examples/blob/main/reinforcement_learning/reinforce.py">reinforce.py</a>.
However, feel free to expeiment with this.

@code{.cpp}
const uint_t L1 = 4;
const uint_t L2 = 128;
const uint_t L3 = 2;
const real_t LEARNING_RATE = 0.01;


// The class that models the Policy network to train
class PolicyNetImpl: public torch::nn::Module
{
public:

	///
	/// \brief Constructor
	///
    PolicyNetImpl();
	
	// To execute the network in C++, 
	// we simply call the forward() method
    torch_tensor_t forward(torch_tensor_t state);

	///
	/// \brief act Every policy network should expose
	/// an act function that takes a StateTp and returns
	/// an std::tuple<uint_t, torch_tensor_t>
	///
    template<typename StateTp>
    std::tuple<uint_t, torch_tensor_t> act(const StateTp& state);
	
private:

   torch::nn::Linear fc1_;
   torch::nn::Dropout dp_;
   torch::nn::Linear fc2_;
   bool is_playing_{false};
};


PolicyNetImpl::PolicyNetImpl()
    :
      fc1_(torch::nn::Linear(L1, L2)),
	  dp_(torch::nn::Dropout(0.6)),
      fc2_(torch::nn::Linear(L2, L3))
{
    register_module("fc1", fc1_);
	register_module("dp", dp_);
    register_module("fc2", fc2_);
}


torch_tensor_t
PolicyNetImpl::forward(torch_tensor_t x){

	x = fc1_->forward(x);
	
	if(!is_playing_){
		x = dp_ -> forward(x);
		
	}
    x = F::relu(x);
    x = fc2_->forward(x);
    return F::softmax(x,  F::SoftmaxFuncOptions(0));
}


template<typename StateTp>
std::tuple<uint_t, torch_tensor_t>
PolicyNetImpl::act(const StateTp& state){

    auto torch_state = torch::tensor(state);
    auto probs = forward(torch_state);
	
    auto m = TorchCategorical(probs, false);
	
    auto action = m.sample();
    return std::make_tuple(action.item().toLong(), 
	                       m.log_prob(action));

}

TORCH_MODULE(PolicyNet);
@endcode

The _ReinforceSolver_ expects a policy type that exposes an _act_ function that
returns the action type and the log probability of taking this action. 

Note also that the network above uses  _torch::nn::Dropout_. Dropout is some form of
regularization that we should only use during training. 


## Driver code

@code{.cpp}
#include "cubeai/base/cubeai_config.h"

#ifdef USE_PYTORCH

#include "cubeai/base/cubeai_types.h"
#include "cubeai/rl/algorithms/pg/reinforce.h"
#include "cubeai/rl/algorithms/pg/reinforce_config.h"
#include "cubeai/rl/trainers/rl_serial_agent_trainer.h"
#include "cubeai/maths/optimization/optimizer_type.h"
#include "cubeai/maths/optimization/pytorch_optimizer_factory.h"
#include "cubeai/maths/statistics/distributions/torch_categorical.h"
# include "cubeai/utils/torch_adaptor.h"


#include "rlenvs/utils/io/csv_file_writer.h"
#include "rlenvs/envs/api_server/apiserver.h"
#include "rlenvs/envs/gymnasium/classic_control/cart_pole_env.h"

#include <boost/log/trivial.hpp>

#include <torch/torch.h>

#include <unordered_map>
#include <iostream>
#include <string>
#include <any>
#include <filesystem>
#include <map>

namespace rl_example_13{


const std::string SERVER_URL = "http://0.0.0.0:8001/api";
const std::string EXPERIMENT_ID = "2";
const std::string POLICY = "policy.csv";

namespace F = torch::nn::functional;

using bitrl::real_t;
using bitrl::uint_t;
using bitrl::float_t;
using bitrl::int_t;
using bitrl::torch_tensor_t;
using bitrl::DeviceType;
using bitrl::rl::algos::pg::ReinforceSolver;
using bitrl::rl::algos::pg::ReinforceConfig;
using bitrl::rl::RLSerialAgentTrainer;
using bitrl::rl::RLSerialTrainerConfig;
using bitrl::maths::stats::TorchCategorical;
using rlenvscpp::envs::RESTApiServerWrapper;	
using rlenvscpp::envs::gymnasium::CartPole;


const uint_t L1 = 4;
const uint_t L2 = 128;
const uint_t L3 = 2;
const real_t LEARNING_RATE = 0.01;


// The class that models the Policy network to train
class PolicyNetImpl: public torch::nn::Module
{
public:

	///
	/// \brief Constructor
	///
    PolicyNetImpl();
	
	// To execute the network in C++, 
	// we simply call the forward() method
    torch_tensor_t forward(torch_tensor_t state);

	///
	/// \brief act Every policy network should expose
	/// an act function that takes a StateTp and returns
	/// an std::tuple<uint_t, torch_tensor_t>
	///
    template<typename StateTp>
    std::tuple<uint_t, torch_tensor_t> act(const StateTp& state);
	
private:

   torch::nn::Linear fc1_;
   torch::nn::Dropout dp_;
   torch::nn::Linear fc2_;
   bool is_playing_{false};
};


PolicyNetImpl::PolicyNetImpl()
    :
      fc1_(torch::nn::Linear(L1, L2)),
	  dp_(torch::nn::Dropout(0.6)),
      fc2_(torch::nn::Linear(L2, L3))
{
    register_module("fc1", fc1_);
	register_module("dp", dp_);
    register_module("fc2", fc2_);
}


torch_tensor_t
PolicyNetImpl::forward(torch_tensor_t x){

	x = fc1_->forward(x);
	
	if(!is_playing_){
		x = dp_ -> forward(x);
		
	}
    x = F::relu(x);
    x = fc2_->forward(x);
    return F::softmax(x,  F::SoftmaxFuncOptions(0));
}


template<typename StateTp>
std::tuple<uint_t, torch_tensor_t>
PolicyNetImpl::act(const StateTp& state){

    auto torch_state = torch::tensor(state);
    auto probs = forward(torch_state);
	
    auto m = TorchCategorical(probs, false);
	
    auto action = m.sample();
    return std::make_tuple(action.item().toLong(), 
	                       m.log_prob(action));

}

TORCH_MODULE(PolicyNet);

typedef CartPole env_type;
typedef PolicyNet policy_type;
typedef ReinforceSolver<env_type, PolicyNet> solver_type;
}


int main(){
	
	BOOST_LOG_TRIVIAL(info)<<"Starting agent training";
    using namespace rl_example_13;

    try{

        // let's create a directory where we want to
        //store all the results from running a simualtion
        std::filesystem::create_directories("experiments/" + EXPERIMENT_ID);
        torch::manual_seed(42);

		BOOST_LOG_TRIVIAL(info)<<"Creating environment...";
		
		RESTApiServerWrapper server(SERVER_URL, true);
        auto env = CartPole(server);
	
        std::unordered_map<std::string, std::any> options;

        // with Gymnasium v0 is not working
        env.make("v1", options);
        env.reset();

        BOOST_LOG_TRIVIAL(info)<<"Done...";
		BOOST_LOG_TRIVIAL(info)<<"Number of actions="<<env.n_actions();

        PolicyNet policy;

		// configuration for the REINFORCE solver
		ReinforceConfig opts;
		
		const auto N_EPISODES = 2000;
								
		opts.gamma = 0.999;
		opts.normalize_rewards = false;
		opts.max_itrs_per_episode = 500; // the max we can get according to docs
		opts.n_episodes = N_EPISODES;
		opts.device_type = DeviceType::CPU;

        std::map<std::string, std::any> opt_options;
        opt_options.insert(std::make_pair("lr", LEARNING_RATE));

		using namespace bitrl::maths;
        auto pytorch_ops = optim::pytorch::build_pytorch_optimizer_options(optim::OptimzerType::ADAM,
																		   opt_options);

        auto policy_optimizer = optim::pytorch::build_pytorch_optimizer(optim::OptimzerType::ADAM,
																		*policy, pytorch_ops);

		
        solver_type solver(opts, policy, policy_optimizer);
		
        RLSerialTrainerConfig config;
        config.n_episodes = N_EPISODES;
        config.output_msg_frequency = 20;
        RLSerialAgentTrainer<env_type, solver_type> trainer(config, solver);
		
		
        trainer.train(env);

        auto info = trainer.train(env);
        BOOST_LOG_TRIVIAL(info)<<"Training info...";
		BOOST_LOG_TRIVIAL(info)<<info;

        // save the rewards per episode for visualization
        // purposes
		auto experiment_path = std::string("experiments/") + EXPERIMENT_ID;
        
        // save the policy also so that we can load it and check
        // use it
        auto policy_model_filename = experiment_path + std::string("/reinforce_cartpole_policy.pth");
        
		
		torch::save(policy, policy_model_filename + std::string("policy.pth"));
		
		// or we can use serialize
        torch::serialize::OutputArchive archive;
        policy->save(archive);
        archive.save_to(policy_model_filename);
		
		// write the loss values
		auto& loss_vals  = solver.get_monitor().policy_loss_values;
		
		
		BOOST_LOG_TRIVIAL(info)<<"Loss values size: "<<loss_vals.size();
		rlenvscpp::utils::io::CSVWriter loss_csv_writer(experiment_path + "/" + "loss.csv",
														  rlenvscpp::utils::io::CSVWriter::default_delimiter());
		loss_csv_writer.open();
		
		auto episode_counter = 0;
		for(uint_t i=0; i<loss_vals.size(); ++i){
			std::tuple<uint_t, real_t> row = {episode_counter++, loss_vals[i]};
			loss_csv_writer.write_row(row);
		}
		
		loss_csv_writer.close();
		
		
		auto& rewards  = solver.get_monitor().rewards;
		rlenvscpp::utils::io::CSVWriter rewards_csv_writer(experiment_path + "/" + "rewards.csv",
														  rlenvscpp::utils::io::CSVWriter::default_delimiter());
		rewards_csv_writer.open();
		
		episode_counter = 0;
		
		for(uint_t i=0; i<rewards.size(); ++i){
			std::tuple<uint_t, real_t> row = {episode_counter++, rewards[i]};
			rewards_csv_writer.write_row(row);
		}
		
		rewards_csv_writer.close();
		
		
		auto& episode_duration  = solver.get_monitor().episode_duration;
		rlenvscpp::utils::io::CSVWriter episode_duration_csv_writer(experiment_path + "/" + "episode_duration.csv",
														            rlenvscpp::utils::io::CSVWriter::default_delimiter());
		episode_duration_csv_writer.open();
		
		episode_counter = 0;
		for(uint_t i=0; i<episode_duration.size(); ++i){
			std::tuple<uint_t, real_t> row = {episode_counter++, episode_duration[i]};
			episode_duration_csv_writer.write_row(row);
		}
		
		episode_duration_csv_writer.close();
		
		BOOST_LOG_TRIVIAL(info)<<"Finished agent training";
		
    }
    catch(std::exception& e){
        std::cout<<e.what()<<std::endl;
    }
    catch(...){
        std::cout<<"Unknown exception occured"<<std::endl;
    }
    return 0;
}
#else
#include <iostream>
int main(){

    std::cout<<"This example requires PyTorch. Reconfigure bitrl with USE_PYTORCH fag turned ON."<<std::endl;
    return 0;
}
#endif

@endcode

Running the driver above produces the following plots.


| ![reinforce-reward](images/reinforce_reward.png) |
|:--:|
| **Figure 1: Undiscounted total reward for REINFORCE over training.**|

| ![reinforce-loss](images/reinforce_loss.png) |
|:--:|
| **Figure 2: Loss for REINFORCE over training.**|

| ![reinforce-episode-duration](images/reinforce_episode_duration.png) |
|:--:|
| **Figure 3: Episode duration for REINFORCE over training.**|

## Summary

This example introduced the ```ReinforceSolver``` class that models the REINFORCE algorithm.

The RINFORCE algorithm, just like all policy gradient methods, suffers from high variance in the gradient estimation.
There are several reasons behind this high variance; e.g. sparse rewards or environment randomness.
Regardless of reasons behind the high variance in the gradient, 
this effect can be detrimental during learning as it destabilizes it. 
Hence, reducing the high variance is important for feasible training. Using a bseline
is one approach we can use in order to reduce the variance in the gradient approximation.
We will use this in <a href="https://github.com/pockerman/bitrl/blob/master/examples/rl/rl_example_20/rl_example_20.md">Example 20: REINFORCE with baseline algorithm on CartPole</a>.
