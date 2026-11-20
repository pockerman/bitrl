\page cuberl_example_14 CubeRL Example 14: Value iteration on FrozenLake

\ref cuberl_example_13 solved _FrozenLake_ with policy iteration. This example solves it with value iteration,
and then plays the policy it finds. The environment is the 4x4 _FrozenLake_ from
<a href="https://gymnasium.farama.org/environments/toy_text/frozen_lake/">Gymnasium</a>, made with
`is_slippery` set to `false`, so every action moves the agent where it was meant to. It runs on the
<a href="https://github.com/pockerman/bitrl-rest-api">bitrl-rest-api</a> server, which the example talks to
over HTTP at `http://0.0.0.0:8001/api`, so start the server first.

## Value iteration

Policy iteration alternates between evaluating a policy and improving it. Value iteration folds the two into
one update. It sweeps over the states and replaces the value of each with the value of its best action,

\f[
V(s) \leftarrow \max_{\alpha} \sum_{s', r} p(s', r | s, \alpha) \left[ r + \gamma V(s') \right],
\f]

until a sweep changes no value by more than a tolerance. The policy is then read off the value function: in
each state, take the action with the largest expected return. Like policy iteration, it needs the transition
probabilities and rewards of the environment, which \ref cuberl::envs::gymnasium::FrozenLake "FrozenLake"
provides.

\ref cuberl::rl::algos::dp::ValueIteration "ValueIteration" uses \f$\gamma = 0.99\f$ and a tolerance of
\f$10^{-8}\f$. It updates the values in place, so a state later in a sweep already sees the new values of the
states before it. \ref cuberl::rl::RLSerialAgentTrainer "RLSerialAgentTrainer" runs one sweep per training
episode, at most 10000 of them, and stops as soon as a sweep changes no value by more than the tolerance.

## What it writes

In the directory it runs from, the example writes

| File | What |
| ---- | ---- |
| `value_iteration_frozen_lake_v1.csv` | the value of each state |
| `reward_per_itr.csv` | for each sweep, the largest change of a value in it. `ValueIteration` reports it as the reward of the episode, so that the trainer keeps it |
| `policy.csv` | the greedy policy, the action to take in each state |

`plot.py` draws the value function on the grid of the lake and the largest change per sweep. `play.py` plays
the policy in Gymnasium and draws each step. Both read the files from the directory they run in, or from the
path given as their argument:

@code{.sh}
python plot.py <directory of the csv files>
python play.py <directory of the csv files>/policy.csv
@endcode

`play.py` needs the `gymnasium` Python package with its toy text environments.

## The results

The agent starts in the top left corner, the goal is in the bottom right, and the four cyan squares with a
value of 0 are the holes. The goal itself also has a value of 0: the episode ends there, so no rewards come
after it. Reaching the goal is the only reward, 1, so the value of a state is \f$\gamma^{n-1}\f$, where \f$n\f$
is the number of steps from it to the goal. The start is 6 steps away and has a value of
\f$0.99^5 \approx 0.951\f$.

| ![State value function](frozen_lake_state_value_function.png) |
|:--:|
| **Figure 1: The state value function.** |

The policy reaches the goal in those 6 steps, the shortest path around the holes.

| ![fz-vi-step-1](fz_vi_step_1.png) |
|:--:|
| **Figure 2: Playing FrozenLake, step 1.** |

| ![fz-vi-step-2](fz_vi_step_2.png) |
|:--:|
| **Figure 3: Step 2.** |

| ![fz-vi-step-3](fz_vi_step_3.png) |
|:--:|
| **Figure 4: Step 3.** |

| ![fz-vi-step-4](fz_vi_step_4.png) |
|:--:|
| **Figure 5: Step 4.** |

| ![fz-vi-step-5](fz_vi_step_5.png) |
|:--:|
| **Figure 6: Step 5.** |

| ![fz-vi-step-6](fz_vi_step_6.png) |
|:--:|
| **Figure 7: Step 6, the goal.** |

## The driver code

\include example_14/example_14.cpp
