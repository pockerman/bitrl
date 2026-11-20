\page cuberl_example_16 CubeRL Example 16: Q-learning on CliffWorld

In this example, we will apply another temporal difference learning algorithm namely <a href="https://en.wikipedia.org/wiki/Q-learning">Q-learning</a>
on _CliffWorld_.  This is a Gymnasium-based environment implemented in \ref cuberl::envs::gymnasium::CliffWorld "cuberl::envs::gymnasium::CliffWorld" class.
We used this environment in example \ref cuberl_example_13.

Q-learning, just like SARSA, is a model-free reinforcement learning algorithm that learns how good an action is in a given state, 
without needing a model of the environment.
As the name suggests, the algorithms learns a state-action value function \f$Q(s,\alpha)\f$.
The algorithm is very similar to SARSA. Before we show the algorithmic steps, here is the update function used in Q-learning

$$
Q(s,\alpha) = Q(s,\alpha) + \eta \big[r_t + \gamma \max_{\alpha_{t+1}} Q(s_{t+1}, \alpha_{t+1}) - Q(s,\alpha)\big]
$$

The algorithm just like SARSA looks into the future but always selects the action that provides the best $Q$. Contrast this with the update formula
for SARSA:

$$
Q(s_t, \alpha_t) = Q(s_t, \alpha_t) + \eta \left[r_{t+1} + \gamma Q(s_{t+1}, \alpha_{t+1}) - Q(s_t, \alpha_t) \right]
$$

So Q-learning instead of consulting \f$\pi\f$ about \f$\alpha_{t+1}\f$ it simply acts greedily and selects the action that provides the best state-action value function.
This is called optimistic bootstrapping; assume the best possible action will be taken next, regardless of what we'll actually do.
Here are the steps for Q-learning. These are very similar to SARSA:

#### Step 1

The first step is to initialize the table that represents $Q(s,\alpha)$ to arbitrary values; often this is just zero. This however, can also
be values that encourage exploration.

#### Step 2

The algorithm begins by some be presented with a state. Q-learning needs to decide what to do whilst at this state. This is done using a policy
$\pi$ which most often will be an \f$\epsilon-\f$greedy policy.

#### Step 3

The algorithm will execute the action that was selected from step 2. The environment will respond with a reward \f$r_t\f$ and the new state \f$s_{t+1}\f$.
In order to apply the update formula we need to calculate what is the best \f$Q\f$ at \f$s_{t+1}\f$.
The update rule, see above, is using the future \f$maxQ(s_{t+1}, \alpha_{t+1})\f$ in order to update the current \f$Q(s_{t}, \alpha_{t})\f$
Compared to SARSA, in Q-learning we don't necessarily choose \f$\alpha_{t+1}\f$ when at \f$s_{t+1}\f$ but instead ask the policy to provide the action to.

#### Step 4

After updating, we move to \f$s_{t+1}\f$ and ask \f$\pi\f$ to provide us with an action and we repeat step 3.
If we've reached a terminal state (like the end of a game or a completed transaction), the episode ends and we start fresh.
The environment has to inform the agent about whether it reached the end of the game or not. So when we take an action in the environment,
we will usually receive not just a reward signal and the next state but also a flag indicating if the end of the game or simulation has been reached.

All in all, Q-learning learns the optimal policy. It is a fast and efficient algorithm that is ideal for batch learning.
Q-learning, compared to SARSA, is more optimistic which can be dangerous in risky domains, and it suffers from maximization bias that can overestimate action values when estimates are noisy [2].

The Q-learning algorithm is implemented in the \ref cuberl::rl::algos::td::QLearningSolver "cuberl::rl::algos::td::QLearningSolver"  class


@code{.cpp}
#include "cuberl/base/cuberl_types.h"
#include "cuberl/rl/algorithms/td/q_learning.h"
#include "cuberl/rl/policies/epsilon_greedy_policy.h"
#include "cuberl/rl/trainers/rl_serial_agent_trainer.h"

#include "bitrl/utils/io/csv_file_writer.h"
#include "cuberl/network/rest_rl_env_client.h"
#include "cuberl/envs/gymnasium/toy_text/cliff_world_env.h"

#include <iostream>
#include <iostream>
#include <unordered_map>
#include <boost/log/trivial.hpp>
@endcode

@code{.cpp}
namespace rl_example_10{

const std::string SERVER_URL = "http://0.0.0.0:8001/api";

const std::string SOLUTION_FILE = "qlearning_cliff_walking_v1.csv";
const std::string REWARD_PER_ITR = "reward_per_itr.csv";
const std::string POLICY = "policy.csv";

using cuberl::real_t;
using cuberl::uint_t;
using cuberl::rl::policies::EpsilonGreedyPolicy;
using cuberl::rl::algos::td::QLearningSolver;
using cuberl::rl::algos::td::QLearningConfig;
using cuberl::rl::policies::EpsilonDecayOption;
using cuberl::rl::RLSerialAgentTrainer;
using cuberl::rl::RLSerialTrainerConfig;
using cuberl::network::RESTRLEnvClient;
typedef  cuberl::envs::gymnasium::CliffWorld env_type;

}


int main(){

	BOOST_LOG_TRIVIAL(info)<<"Starting agent training";
    using namespace rl_example_10;

    try{
		
		RESTRLEnvClient server(SERVER_URL, true);

        // create the environment
        env_type env(server);

        BOOST_LOG_TRIVIAL(info)<<"Creating environment...";
        
		std::unordered_map<std::string, std::any> options;
    	std::unordered_map<std::string, std::any> reset_options;
        env.make("v0", options, reset_options);
        env.reset();
		
        BOOST_LOG_TRIVIAL(info)<<"Done...";
        BOOST_LOG_TRIVIAL(info)<<"Number of states="<<env.n_states();
        BOOST_LOG_TRIVIAL(info)<<"Number of actions="<<env.n_actions();

		// create an e-greedy policy. Use the number 
		// of actions as a seed. Use a constant epsilon
        EpsilonGreedyPolicy policy(0.01, env.n_actions(), 
		                           EpsilonDecayOption::NONE);

        QLearningConfig qlearn_config;
        qlearn_config.gamma = 0.9;
        qlearn_config.eta = 0.5;
        qlearn_config.tolerance = 1.0e-8;
        qlearn_config.max_num_iterations_per_episode = 1000;
        qlearn_config.path = SOLUTION_FILE;

        QLearningSolver<env_type, EpsilonGreedyPolicy> algorithm(qlearn_config, policy);

        RLSerialTrainerConfig trainer_config = {10, 1000, 1.0e-8};

        RLSerialAgentTrainer<env_type,
                QLearningSolver<env_type, EpsilonGreedyPolicy>> trainer(trainer_config, algorithm);

        auto info = trainer.train(env);
        BOOST_LOG_TRIVIAL(info)<<info;
		
		// save the reward the agent achieved per training epoch
		auto reward = trainer.episodes_total_rewards();
		auto iterations = trainer.n_itrs_per_episode();
	
		bitrl::utils::io::CSVWriter csv_writer(REWARD_PER_ITR);
		csv_writer.open();
		
		csv_writer.write_column_names({"epoch", "reward"});
		
		auto epoch = static_cast<uint_t>(0);
		for(auto val: reward){
			
			std::tuple<uint_t, real_t> row = {epoch++, val};
			csv_writer.write_row(row);
		}
		
		csv_writer.close();
		
		// build the policy
		algorithm.build_policy().save(POLICY);

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



| ![cw-ql-step-1](images/ql_cliff_step_1.png) |
|:--:|
| **Figure 1:Playing CliffWorld step 1.**|

| ![cw-ql-step-2](images/ql_cliff_step_2.png) |
|:--:|
| **Figure 2:Playing CliffWorld step 2.**|

| ![cw-ql-step-3](images/ql_cliff_step_3.png) |
|:--:|
| **Figure 3:Playing CliffWorld step 3.**|

| ![cw-ql-step-9](images/ql_cliff_step_9.png) |
|:--:|
| **Figure 4:Playing CliffWorld step 9.**|

| ![cw-ql-step-12](images/ql_cliff_step_12.png) |
|:--:|
| **Figure 5:Playing CliffWorld step 12.**|

| ![cw-ql-step-13](images/ql_cliff_step_13.png) |
|:--:|
| **Figure 6:Playing CliffWorld step 13.**|

