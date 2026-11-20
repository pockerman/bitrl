\page cuberl_example_13 CubeRL Example 13: Policy iteration on FrozenLake

This example solves the 4x4 _FrozenLake_ environment from
<a href="https://gymnasium.farama.org/environments/toy_text/frozen_lake/">Gymnasium</a> with policy
iteration. The environment runs on the <a href="https://github.com/pockerman/bitrl-rest-api">bitrl-rest-api</a>
server, which the example talks to over HTTP at `http://0.0.0.0:8001/api`, so start the server first.

Policy iteration is a dynamic programming method: it needs a model of the environment, the transition
probabilities and rewards, which \ref cuberl::envs::gymnasium::FrozenLake "FrozenLake" provides. Starting
from a policy, it repeats two steps until the policy stops changing:

- **Policy evaluation.** Compute the state value function \f$V_{\pi}\f$ of the current policy \f$\pi\f$ by
  sweeping over the states with the Bellman expectation equation
  \f$V_{\pi}(s) = \sum_{\alpha} \pi(\alpha | s) \sum_{s', r} p(s', r | s, \alpha) \left[ r + \gamma V_{\pi}(s') \right]\f$.
- **Policy improvement.** Make the policy greedy with respect to \f$V_{\pi}\f$: in each state, pick the action
  with the largest expected return.

The example starts from a \ref cuberl::rl::policies::UniformDiscretePolicy "UniformDiscretePolicy", which picks
every action with the same probability. \ref cuberl::rl::algos::dp::PolicyIterationSolver "PolicyIterationSolver"
uses \f$\gamma = 1\f$ and runs 15 sweeps of policy evaluation, with a tolerance of \f$10^{-8}\f$, in every
iteration. \ref cuberl::rl::RLSerialAgentTrainer "RLSerialAgentTrainer" runs at most 100 iterations, and stops
earlier once policy improvement leaves the policy unchanged. The environment is made with its default options,
so the lake is slippery: an action moves the agent in the intended direction only some of the time. The state
value function is then saved to `policy_iteration_frozen_lake_v0.csv`.

`plot.py` draws the state value function on the 4x4 grid. It reads `policy_iteration.csv`, so rename the file
the example writes or pass its name to `main`.

## The driver code

\include example_13.cpp
