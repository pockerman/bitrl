\page bitrl_example_17 bitrl Example 17: An A2C Solver for CartPole

\ref bitrl_example_19
introduced the vanilla REINFORCE algorithm in order to solve the pole balancing problem. 
The REINFORCE algorithm is a simple algorithm and easy to use however it exhibits a large variance in the reward signal.

In this exampel, we will introduce <a href="https://proceedings.neurips.cc/paper_files/paper/1999/file/6449f44a102fde848669bdd9eb6b76fa-Paper.pdf">actor-critic</a> methods
and specifically the A2C algorithm.	The main objective of actor-critic methods is to further reduce the high gradient variance. 
One way towards this direction is to use the so-called reward-to-go term for a trajectory \f$T\f$. 

\f[G = \sum_{k = t}^T R(s_{k}, \alpha_{k})\f]

The environment runs on the <a href="https://github.com/pockerman/bitrl-rest-api">bitrl-rest-api</a> server, so start it first.


The workings of the A2C algorithm are handled by the \ref bitrl::rl::algos::pg::A2CSolver "A2CSolver"
class. The class accepts three template parameters

- The environment type 
- The policy type or the actor type 
- The critic type

The role of the actor is to select an action for the agent to take.
The role of the critic is to tell us whether that action was good or bad.
We could use of course the raw reward signal to have an assessment on this
but for enviroments where the reward is sparse this may not work or for enviroments
where the reward signal is the same for most actions.

Given this approach, the critic will be a term in the actor’s loss function,
whilst the critic will learn directly from the provided reward signals.

In the code below, the critic and action networks do not share any details.
This need not be the case however. You can come up with an implementation
where the two networks share most of the layers and only differentiate
at the output layer.


### The Actor network class

Just like in \ref bitrl_example_19,
the actor class has to expose an act function with the following signature:

@code{.cpp}
template<typename StateTp>
std::tuple<uint_t, torch_tensor_t> act(const StateTp& state)
@endcode

Below is the class that implements the actor model. It is identical to the one from the REINFORCE example.

@code{.cpp}
class ActorNetImpl: public torch::nn::Module
{
public:

    
    ActorNetImpl();
	torch_tensor_t forward(torch_tensor_t state);
	template<typename StateTp>
	std::tuple<uint_t, torch_tensor_t> act(const StateTp& state);
			   
private:

    torch::nn::Linear fc1_;
	torch::nn::Dropout dp_;
    torch::nn::Linear  fc2_;

};

ActorNetImpl::ActorNetImpl()
:
torch::nn::Module(),
fc1_(torch::nn::Linear(L1, L2)),
dp_(torch::nn::Dropout(0.6)),
fc2_(torch::nn::Linear(L2, L3))
{
   register_module("fc1_", fc1_);
   register_module("dp_",  dp_);
   register_module("fc2_", fc2_);
}

template<typename StateTp>
std::tuple<uint_t, torch_tensor_t> 
ActorNetImpl::act(const StateTp& state){
	
	auto torch_state = torch::tensor(state);
    auto probs = forward(torch_state);
	
	auto m = TorchCategorical(probs, false);
	
	auto action = m.sample();
    return std::make_tuple(action.item().toLong(), 
	                       m.log_prob(action));
}


torch_tensor_t
ActorNetImpl::forward(torch_tensor_t state){
	
	state = fc1_->forward(state);
	state = dp_ -> forward(state);
		
	state = F::relu(state);
    state = fc2_->forward(state);
    return F::softmax(state,  F::SoftmaxFuncOptions(0));
}
@endcode


### The Critic network class

We also need to provide a critic network. The class has to expose an ```evaluate``` function
with the following signature:

@code{.cpp}
template<typename StateTp>
torch_tensor_t evaluate(const StateTp& state); 
@endcode


@code{.cpp}
/// The Critic network 
class CriticNetImpl: public torch::nn::Module
{
public:

    CriticNetImpl();
    torch_tensor_t forward(torch_tensor_t state);
	
	template<typename StateTp>
	torch_tensor_t evaluate(const StateTp& state); 

private:

    torch::nn::Linear fc1_;
    torch::nn::Linear fc2_;
    torch::nn::Linear fc3_;

};


CriticNetImpl::CriticNetImpl()
:
torch::nn::Module(),
fc1_(torch::nn::Linear(4, 128)),
fc2_(torch::nn::Linear(128, 256)),
fc3_(torch::nn::Linear(256, 1))
{
   register_module("fc1_", fc1_);
   register_module("fc2_", fc2_);
   register_module("fc3_", fc3_);
}

torch_tensor_t
CriticNetImpl::forward(torch_tensor_t state){

    auto output = F::relu(fc1_ -> forward(state));
    output = F::relu(fc2_ -> forward(output));
    output = fc3_ -> forward(output);
    return output;
}

template<typename StateTp>
torch_tensor_t 
CriticNetImpl::evaluate(const StateTp& state){
	
	auto torch_state = torch::tensor(state);
	return forward(torch_state);
}
@endcode


## The driver code

The driver code for this tutorial is shown below. 


\include example_17/example_17.cpp

Running the driver above produces the following plots.


| ![reinforce-reward](images/episode_rewards.png) |
|:--:|
| **Figure 1: Undiscounted total reward for A2C over training.**|

| ![reinforce-loss](images/policy_loss.png) |
|:--:|
| **Figure 2: Actor loss for A2C over training.**|


| ![reinforce-loss](images/critic_loss.png) |
|:--:|
| **Figure 3: Critic loss for A2C over training.**|

| ![reinforce-episode-duration](images/episode_duration.png) |
|:--:|
| **Figure 4: Episode duration for A2C over training.**|

