\page bitrl_example_16 bitrl Example 16: Q-learning on CliffWorld

In this example, we will apply another temporal difference learning algorithm namely <a href="https://en.wikipedia.org/wiki/Q-learning">Q-learning</a>
on _CliffWorld_.  This is a Gymnasium-based environment implemented in \ref bitrl::envs::gymnasium::CliffWorld "bitrl::envs::gymnasium::CliffWorld" class.
We used this environment in \ref bitrl_example_15.

Q-learning, just like SARSA, is a model-free reinforcement learning algorithm that learns how good an action is in a given state, 
without needing a model of the environment.
As the name suggests, the algorithms learns a state-action value function \f$Q(s,\alpha)\f$.
The algorithm is very similar to SARSA. Before we show the algorithmic steps, here is the update function used in Q-learning

\f[
Q(s,\alpha) = Q(s,\alpha) + \eta \big[r_t + \gamma \max_{\alpha_{t+1}} Q(s_{t+1}, \alpha_{t+1}) - Q(s,\alpha)\big]
\f]

The algorithm just like SARSA looks into the future but always selects the action that provides the best \f$Q\f$. Contrast this with the update formula
for SARSA:

\f[
Q(s_t, \alpha_t) = Q(s_t, \alpha_t) + \eta \left[r_{t+1} + \gamma Q(s_{t+1}, \alpha_{t+1}) - Q(s_t, \alpha_t) \right]
\f]

So Q-learning instead of consulting \f$\pi\f$ about \f$\alpha_{t+1}\f$ it simply acts greedily and selects the action that provides the best state-action value function.
This is called optimistic bootstrapping; assume the best possible action will be taken next, regardless of what we'll actually do.
Here are the steps for Q-learning. These are very similar to SARSA:

#### Step 1

The first step is to initialize the table that represents \f$Q(s,\alpha)\f$ to arbitrary values; often this is just zero. This however, can also
be values that encourage exploration.

#### Step 2

The algorithm begins by some be presented with a state. Q-learning needs to decide what to do whilst at this state. This is done using a policy
\f$\pi\f$ which most often will be an \f$\epsilon-\f$greedy policy.

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

The Q-learning algorithm is implemented in the \ref bitrl::rl::algos::td::QLearningSolver "bitrl::rl::algos::td::QLearningSolver"  class


@code{.cpp}
#include "bitrl/base/bitrl_types.h"
#include "bitrl/rl/algorithms/td/q_learning.h"
#include "bitrl/rl/policies/epsilon_greedy_policy.h"
#include "bitrl/rl/trainers/rl_serial_agent_trainer.h"

#include "bitsim/utils/io/csv_file_writer.h"
#include "bitrl/network/rest_rl_env_client.h"
#include "bitrl/envs/gymnasium/toy_text/cliff_world_env.h"

#include <iostream>
#include <iostream>
#include <unordered_map>
#include <boost/log/trivial.hpp>
@endcode

\include example_16/example_16.cpp



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

