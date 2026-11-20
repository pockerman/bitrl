\page bitrl_example_acrobot_vector_env A vector environment: Acrobot

Reinforcement learning collects experience faster from many copies of an environment at once.
\ref bitrl::envs::gymnasium::AcrobotV "AcrobotV" runs several copies of Gymnasium's Acrobot, a
two-link pendulum that has to swing its tip above a line, on the
<a href="https://github.com/pockerman/bitrl-rest-api">bitrl-rest-api</a> server, and steps them
all with one call.

## What you need

The bitrl-rest-api server running on `http://0.0.0.0:8001`; see its README. See
\ref bitrl_example_gymnasium_part_1 for how bitrl talks to it.

## Running the example

@code{.sh}
./acrobot_vector_env
@endcode

The example makes three copies with the option `num_envs`, resets them with the seed 42, steps them
with one action each, 0, 1 and 2, and prints the time steps.

## The code

\include acrobot_vector_env.cpp
