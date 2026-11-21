\page bitrl_example_sarsa SARSA on CliffWorld

In this example we will apply the SARSA algorithm on <a href="https://gymnasium.farama.org/environments/toy_text/cliff_walking/">CliffWorld</a>.
This is a Gymnasium-based environment implemented in \ref bitrl::envs::gymnasium::CliffWorld "bitrl::envs::gymnasium::CliffWorld" class.
All Gymnasium-based environments require an instance of the <a href="https://github.com/pockerman/bitrl-rest-api">bitsim-envs-api</a> server to run.


Temporal Difference Learning is a combination of dynamic programming and Monte Carlo. TDL does not use the environment transition probabilities and therefore is
a model-free algorithm. One problem with TDL is that we don't know which action to take; TDL approximate that state-value function \f$V(s)\f$
however, we cannot use the Bellman equations to establish the state-action value function \f$Q(s, \alpha)\f$ as we don't know the transition probabilities.
The solution thus is to approximate \f$Q(s, \alpha)\f$ directly. In this note we will discuss one such approach i.e. SARSA or State-Action-Reward-State-Action.

SARSA is  an on-policy method i.e. the agent takes actions and learns, that is it chooses the next \f$Q(s,\alpha)\f$,  under the policy \f$\pi\f$ it currently follows. 
When we update the value of \f$Q(s,\alpha)\f$ we take into account the value \f$Q(s_{t+1},\alpha_{t+1})\f$. 
Hence, with the SARSA algorithm we work with the following tuple \f$(s_t, \alpha_t, s_{t+1}, \alpha_{t+1})\f$.

----
**Remark**

Typically, the policy \f$\pi\f$ will an \f$\epsilon\f$-greedy policy but this need not be the case.

----

The following is the update formula that SARSA is using:

\f[
Q(s_t, \alpha_t) = Q(s_t, \alpha_t) + \eta \left[r_{t+1} + \gamma Q(s_{t+1}, \alpha_{t+1}) - Q(s_t, \alpha_t)\right]
\f]

The state-action value function that is learnt by SARSA reflect a real policy that includes both exploration noise and operational constraints.
The algorithm, compared to Q-learning, is more conservative and therefore convergence is slower. 

Let's have a closer look into the algorithm.

#### Step 1

Reinforcement learning algorithms will usually start by knowing nothing about the environment. So the first step is
to initialize the table that represents \f$Q(s,\alpha)\f$ to arbitrary values; often this is just zero. This however, can also
be values that encourage exploration.

#### Step 2

The algorithm begins by some be presented with a state. SARSA needs to decide what to do whilst at this state. This is done using a policy
\f$\pi\f$ which most often will be an \f$\epsilon-\f$greedy policy.

#### Step 3

The algorithm will execute the action that was selected from step 2. The environment will respond with a reward \f$r_t\f$ and the new state \f$s_{t+1}\f$.
However, we cannot update yet the table \f$Q(s,\alpha)\f$; we need to determine what we will do next. It uses the policy \f$\pi\f$ at the new state \f$s_{t+1}\f$
in order to select the new action \f$\alpha_{t+1}\f$. \f$\alpha_{t+1}\f$ is the action that the agent will take next. 
**This is the key SARSA step that distinguishes it from Q-learning.** [1]. We can now update the state-action value function.
The update rule, see above, is using the future \f$Q(s_{t+1}, \alpha_{t+1})\f$ in order to update the current \f$Q(s_{t}, \alpha_{t})\f$

#### Step 4

After updating, we move to \f$s_{t+1}\f$ and take action \f$\alpha_{t+1}\f$ and repeat step 3. 
If we've reached a terminal state (like the end of a game or a completed transaction), the episode ends and we start fresh.
The environment has to inform the agent about whether it reached the end of the game or not. So when we take an action in the environment,
we will usually receive not just a reward signal and the next state but also a flag indicating if the end of the game or simulation has been reached. 

The SARSA algorithm is implemented in the \ref bitrl::rl::algos::td::SarsaSolver "bitrl::rl::algos::td::SarsaSolver"  class

Below is the driver code for this example. 

\include sarsa/sarsa.cpp


| ![cw-sarsa-step-1](images/sarsa_cliff_step_1.png) |
|:--:|
| **Figure 1:Playing CliffWorld step 1.**|

| ![cw-sarsa-step-3](images/sarsa_cliff_step_3.png) |
|:--:|
| **Figure 2:Playing CliffWorld step 3.**|

| ![cw-sarsa-step-5](images/sarsa_cliff_step_5.png) |
|:--:|
| **Figure 3:Playing CliffWorld step 5.**|

| ![cw-sarsa-step-11](images/sarsa_cliff_step_11.png) |
|:--:|
| **Figure 4:Playing CliffWorld step 11.**|

| ![cw-sarsa-step-14](images/sarsa_cliff_step_14.png) |
|:--:|
| **Figure 5:Playing CliffWorld step 14.**|

| ![cw-sarsa-step-17](images/sarsa_cliff_step_17.png) |
|:--:|
| **Figure 6:Playing CliffWorld step 17.**|

