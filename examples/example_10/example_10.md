\page cuberl_example_10 CubeRL Example 10: Multi-armed bandits

In this example we will look at how to solve the <a href="https://en.wikipedia.org/wiki/Multi-armed_bandit">multi-armed bandits problem</a>.
In particular, we will look at how to use two specific algorithms namely 
\f$\epsilon\f$-greedy and <a href="https://en.wikipedia.org/wiki/Thompson_sampling">Thompson sampling</a>.

The multi-armed bandits framework has a lot of applications to online decision-making such as 
advertisement placement, and recommendation engines. 
An algorithm that solves the problem essentially provides a policy that optimizes
for a given criteria e.g. reward accumulated over how to choose amongst the available options.


----
**Remark**

The article, by  James LeDoux, <a href="https://jamesrledoux.com/algorithms/bandit-algorithms-epsilon-ucb-exp-python/">
Multi-Armed Bandits in Python: Epsilon Greedy, UCB1, Bayesian UCB, and EXP3</a>
provides a nice overview of $\epsilon$-greedy and Thompson sampling whilst the book 
<a href="https://www.oreilly.com/library/view/bandit-algorithms-for/9781449341565/">Bandit Algorithms for Website Optimization</a>
by John Myles White, discusses bandit algorithms for website optimization</a>.

----

\f$\epsilon\f$-greedy sampling

Perhaps one of the simplest approaches to solve the bandits problem is an 
\f$\epsilon\f$-greedy sampling strategy. As probably the name suggests, the algorithm
most of the time will greedily select the action with the highest accumulated reward.
However for a small probability, \f$\epsilon\f$, the algorithm will randomly select an action
amonst the available ones. This is a very simple algorithm and often provides quite good results.

The \f$\epsilon\f$-greedy sampling is implemented in the class  \ref cuberl::rl::policies::EpsilonGreedyPolicy "cuberl::rl::policies::EpsilonGreedyPolicy".

### Thompson sampling

According to wikipedia, Thompson sampling is an approach for selecting actions that 
simultaneously address the <a href="https://en.wikipedia.org/wiki/Exploration%E2%80%93exploitation_dilemma">exploration–exploitation dilemma</a>
in the multi-armed bandit problem. When using Thompson sampling we choose the action that maximizes the expected reward with respect to a randomly drawn belief. 

In the implementation below, we will use the <a href="https://en.wikipedia.org/wiki/Beta_distribution">Beta distribution</a>
but this need not be the case. Feel free to experiment with this.


The class \ref cuberl::envs::bandits::MultiArmedBandits "cuberl::envs::bandits::MultiArmedBandits" conveniently wraps the multi-armed bandits
framework into an environment we can utilise.


## Driver code

@code{.cpp}
#include "cubeai/base/cubeai_types.h"
#include "cubeai/maths/vector_math.h"
#include "cubeai/rl/policies/epsilon_greedy_policy.h"
#include "bitrl/utils/maths/statistics/distributions/beta_dist.h"
#include "bitrl/utils/maths/statistics/distributions/bernoulli_dist.h"
#include "cuberl/envs/multi_armed_bandits/multi_armed_bandits.h"

#include <cmath>
#include <utility>
#include <tuple>
#include <iostream>
#include <random>
#include <algorithm>
#include <numeric>
#include <utility>
#include <unordered_map>

namespace rl_example_2
{

using cuberl::real_t;
using cuberl::uint_t;
using cuberl::int_t;
using cuberl::DynMat;
using cuberl::DynVec;
using cuberl::rl::policies::EpsilonGreedyPolicy;
using bitrl::utils::maths::stats::BetaDist;
using bitrl::utils::maths::stats::BernoulliDist;
using cuberl::envs::bandits::MultiArmedBandits;

// Number of episodes to play
uint_t N_EPISODES = 1000;
real_t EPS = 0.2;

struct SamplingResult
{
	real_t total_reward;
	DynVec<uint_t> lever_pulls;
	
};

SamplingResult 
run_epsilon_greedy(MultiArmedBandits& env, real_t eps){
	
	SamplingResult result;
	result.total_reward = 0.0;
	result.lever_pulls.resize(env.n_actions());
	
	DynVec<real_t> max_rewards(env.n_actions());
	for(uint_t a=0; a < env.n_actions(); ++a){
		max_rewards[a] = 0.0;
		result.lever_pulls[a] = 0;
	}
	
	EpsilonGreedyPolicy policy(eps);
	
	for(uint_t e=0; e < N_EPISODES; ++e){
	
		auto action = policy.get_action(max_rewards);
		auto time_step = env.step(action);
		
		// get the reward from the action
		auto reward = time_step.reward();
		
		// update the table
		max_rewards[action] += reward;
		
		result.total_reward += reward;
		result.lever_pulls[action] += 1;
	}
	return result;	
}

SamplingResult 
run_thompson_sampling(MultiArmedBandits& env){
	
	SamplingResult result;
	result.total_reward = 0.0;
	result.lever_pulls.resize(env.n_actions());
	
	for(uint_t a=0; a < env.n_actions(); ++a){
		result.lever_pulls[a] = 0;
	}
	
	// every arm is modelled using beta dist
	std::vector<BetaDist<real_t>> beta_dists(env.n_actions(), 
	                                 BetaDist<real_t>(1.0, 1.0));
	
	std::vector<real_t> samples(env.n_actions(), 0.0);
	
	for(uint_t e=0; e < N_EPISODES; ++e){
		
		env.reset();
		
		// choose which lever to pull
		for(uint_t i=0; i < beta_dists.size(); ++i){
			samples[i] = beta_dists[i].sample();
		}
		
		// find the max
		auto max_item = cuberl::maths::arg_max(samples);
		auto time_step = env.step(max_item);
		
		auto reward = time_step.reward();
		result.total_reward += reward;
		
		auto alpha = beta_dists[max_item].alpha();
		auto beta =  beta_dists[max_item].beta();
		
		// update the alpha param
		if(reward == env.success_reward()){
			alpha += 1.0;
		}
		else{
			
			beta += 1.0;
		}
		
		// reset the distribution
		beta_dists[max_item].reset(alpha, beta);
		result.lever_pulls[max_item] += 1;
			
	}
	
	return result;
}

}

int main() {

    using namespace rl_example_2;
	
	{
	
		// the environment
		MultiArmedBandits env;
		
		/// all levers have the same probability of success
		// so there shouldn't be any preferance
		std::vector<real_t> p(5, 0.5);
		std::unordered_map<std::string, std::any> options;
		options["p"] = std::any(p);
		env.make("v0", options);
		env.reset();
		
		std::cout<<"Running thompson sampling"<<std::endl;
		auto thompson_result = run_thompson_sampling(env);
		
		std::cout<<"Thompson reward: "<<thompson_result.total_reward<<std::endl;
		std::cout<<"Thompson selected levers: "<<thompson_result.lever_pulls<<std::endl;
		
		std::cout<<"Running epsilon greedy sampling"<<std::endl;
		auto epsilon_greedy_result = run_epsilon_greedy(env,EPS);
		
		std::cout<<"epsilon-greedy reward: "<<epsilon_greedy_result.total_reward<<std::endl;
		std::cout<<"epsilon-greedy selected levers: "<<epsilon_greedy_result.lever_pulls<<std::endl;
	}
	
	std::cout<<"========================================"<<std::endl;
	{
	
		// the environment
		MultiArmedBandits env;
		
		// levers have different probability
		// of success 
		std::vector<real_t> p(5, 0.2);
		p[0] = 0.1;
		p[1] = 0.2;
		p[2] = 0.3;
		p[3] = 0.4;
		p[4] = 0.5;
		std::unordered_map<std::string, std::any> options;
		options["p"] = std::any(p);
		env.make("v0", options);
		env.reset();
		
		std::cout<<"Running thompson sampling"<<std::endl;
		auto thompson_result = run_thompson_sampling(env);
		
		std::cout<<"Thompson reward: "<<thompson_result.total_reward<<std::endl;
		std::cout<<"Thompson selected levers: "<<thompson_result.lever_pulls<<std::endl;
		
		std::cout<<"Running epsilon greedy sampling"<<std::endl;
		auto epsilon_greedy_result = run_epsilon_greedy(env,EPS);
		
		std::cout<<"epsilon-greedy reward: "<<epsilon_greedy_result.total_reward<<std::endl;
		std::cout<<"epsilon-greedy selected levers: "<<epsilon_greedy_result.lever_pulls<<std::endl;
	}

   return 0;
}

@endcode

Running the driver above produces the following output (this may be different on your machine.

@code
Running thompson sampling
Thompson reward: 484
Thompson selected levers: 306 158 256 250  30
Running epsilon greedy sampling
epsilon-greedy reward: 486
epsilon-greedy selected levers: 840  49  36  34  41
========================================
Running thompson sampling
Thompson reward: 447
Thompson selected levers:  19   9  41 229 702
Running epsilon greedy sampling
epsilon-greedy reward: 140
epsilon-greedy selected levers: 840  36  45  37  42
@endcode


We can see that when all levers have the same probability of success Thompson sampling and
\f$\epsilon\f$-greedy perform more or less the same, with \f$\epsilon\f$-greedy having some better performance.
Notice also that Thompson sampling somehow spreads the decision-making amongst the equally good levers.
However, when the levers have different probability of success, Thompson sampling outperforms
\f$\epsilon\f$-greedy by roughly a factor of 4.  Also notice that Thompson sampling favours the last lever
as one would expect. In contrast, \f$\epsilon\f$-greedy seems to favour the lever with the lowest
success probability.

Although we did not implement this here, we can decay \f$\epsilon\f$ over time. This makes sense since as the 
agent spends more time in the game it aquires more knowledge and therefore it does not need to explore as often as
it needs in the early stages. Alternatively, we can have the agent acting completely at random at the beginning
and the then fully exploit.
