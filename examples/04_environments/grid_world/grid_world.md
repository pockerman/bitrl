\page bitrl_example_grid_world A grid world

A grid world is the simplest environment for reinforcement learning: an agent moves on a grid, one
cell a step, and is rewarded for reaching a goal. This example makes
\ref bitrl::envs::grid_world::Gridworld "Gridworld<4>", a 4 x 4 grid, and steps in it. Unlike the
Gymnasium examples it runs in C++, with no server.

## What you need

Nothing beyond bitrl itself.

## The environment

Like every bitsim environment, it is made with `make(version, options, reset_options)` and then
stepped with `step(action)`, which returns a time step with the type of the step, the reward, the
observation and the discount. The actions are the four moves, and every step costs a reward of -1.

The `mode` option places the agent, the goal, a pit and a wall:

- `STATIC`: always in the same cells;
- `RANDOM`: all of them at random, with the `seed` of the reset options.

The observation has one 4 x 4 layer for each of the four things, with a 1 where it is.

## Running the example

@code{.sh}
./grid_world
@endcode

## The code

\include grid_world/grid_world.cpp
