\page bitrl_example_25 bitrl Example 25: PPO on LunarLander

This example trains an agent to land the lander of _LunarLander_ from
<a href="https://gymnasium.farama.org/environments/box2d/lunar_lander/">Gymnasium</a> with proximal policy
optimization (PPO). It uses the continuous version of the environment, `LunarLanderContinuous` in its `v3`
form: the agent sets the throttle of the main engine and of the side engines, two numbers between -1 and 1,
instead of picking one of four discrete actions. The observation has 8 numbers: the position, velocity, angle
and angular velocity of the lander, and whether each of its legs touches the ground.

The environment runs on the <a href="https://github.com/pockerman/bitrl-rest-api">bitrl-rest-api</a> server,
which the example talks to over HTTP at `http://0.0.0.0:8001/api`, so start the server first.

## PPO

PPO is an actor-critic method, like A2C in \ref bitrl_example_17. The actor is the policy, and the critic
estimates the value of a state, which gives the advantage of an action: how much better its return was than the
critic expected. A plain policy gradient step can change the policy so much that it gets worse and does not
recover. PPO limits the step. With the ratio \f$\rho = \pi_{\theta}(\alpha | s) / \pi_{\theta_{old}}(\alpha | s)\f$
of the new and the old probability of an action and its advantage \f$A\f$, it maximises

\f[
\min\left( \rho A, \; \mathrm{clip}(\rho, 1 - \epsilon, 1 + \epsilon) A \right) ,
\f]

so moving \f$\rho\f$ further than \f$\epsilon\f$ from 1 brings no more gain. Because of that it can make several
passes over the same batch of experience.

## The networks

The actor, `ActorNet`, has two hidden layers of 128 and 64 units and two outputs per action: the mean and the
log of the standard deviation of a normal distribution. It samples an action from that distribution and squashes
it into \f$[-1, 1]\f$ with \f$\tanh\f$. The log probability of the action is corrected for the squashing. The
critic, `CriticNet`, has hidden layers of 128 and 256 units and one output, the value of the state.

\ref bitrl::rl::algos::pg::PPOSolver "PPOSolver" trains both with Adam and a learning rate of 0.001, on the CPU.
The example sets 2000 episodes of at most 500 steps each, and leaves the other settings of
\ref bitrl::rl::algos::pg::PPOConfig "PPOConfig" at their defaults: \f$\gamma = 0.99\f$, a clip
\f$\epsilon\f$ of 0.5, 4 passes over each batch, and normalised advantages.

## What it writes

After training, the example writes what the solver recorded to four files in `experiments/1/`, in the directory
it runs from. Each has an index and a value per row:

| File | What |
| ---- | ---- |
| `policy_loss.csv` | the loss of the actor |
| `critic_loss.csv` | the loss of the critic |
| `rewards.csv` | the rewards |
| `episode_duration.csv` | how many steps the episodes lasted |

## The driver code

\include example_25/example_25.cpp
