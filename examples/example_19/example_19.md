\page bitrl_example_19 bitrl Example 19: REINFORCE algorithm on CartPole

The DQN algorithm we used in examples <a href="../rl_example_12/rl_example_12.md">Example 12: DQN algorithm on Gridworld</a>
and <a href="../rl_example_15/rl_example_15.md">Example 15: DQN algorithm on Gridworld with experience replay</a> approximate a value function
and in particular the state-action value function i.e. \f$Q(s, \alpha)\f$. 
However, in reinforcement learning we are more interested in policies rather than value functions
This is because a policy dictates how  an agent behaves in a given state. Indeed one way to 
derive a policy from a state-action value function is to use a greedy approach i.e.

\f[\pi(s) = max_{\alpha} Q(s, \alpha)\f]

In this example, we will implement a policy-based method. In particular, we will use one of the earliest policy-based methods namely the
REINFORCE algorithm. Policy-based methods, as their name probably suggests, approximate policies directly. 
In particular, we will use a neural network to represent the policy \f$\pi\f$ and use variants of the gradient
descent algorithm in order to estimate the weights of the network.

The REINFORCE algorithm samples trajectories from the environment 
using the policy on hand and estimate the gradient using these samples. 
The true gradient can be then estimated using:


\f[ \nabla J(\boldsymbol{\theta}_{t}) = E_{\pi} \left[ G_t \nabla log \pi(\alpha | S_t,  \boldsymbol{\theta}) \right] \f]


where \f$G_t\f$ is the discounted cumulative reward at time step \f$t\f$. 
After the gradient calculation, we update the policy parameters using the following formula

\f[\boldsymbol{\theta} = \boldsymbol{\theta} + \eta \nabla_{\boldsymbol{\theta}} J( \pi_{\boldsymbol{\theta}})\f]

or by substituting the  expected value of the gradient:

\f[\boldsymbol{\theta} = \boldsymbol{\theta} + \eta G_t\nabla_{\boldsymbol{\theta}} log \pi(\alpha | S_t,  \boldsymbol{\theta})\f]

In this example we will use the ```gymnasium.CartPole``` environment.
Specifically, we will use the class \ref bitrl::envs::gymnasium::CartPole "CartPole".
This is a simple class that allows us to
sent HTTP requests to a server that actually runs the environment. We need this as ```gymnasium.CartPole``` is written in Python and we 
want to avoid the complexity of directly executing Python code from the C++ driver.

We will use the policy network from PyTorch examples: <a href="https://github.com/pytorch/examples/blob/main/reinforcement_learning/reinforce.py">reinforce.py</a>.
However, feel free to experiment with this. Note that the PyTorch implementation uses a baseline in the implementation.
We will use this approach in \ref bitrl_example_23


The REINFORCE algorithm is implemented in the class \ref bitrl::rl::algos::pg::ReinforceSolver "ReinforceSolver"
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

\include example_19/example_19.cpp

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
Hence, reducing the high variance is important for feasible training. Using a baseline
is one approach we can use in order to reduce the variance in the gradient approximation.
We will use this in \ref bitrl_example_23.
