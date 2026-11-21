\page bitrl_example_gym_walk Using GymWalk Environment

The \ref bitrl::envs::gdrl::GymWalk "bitrl::envs::gdrl::GymWalk" is an implementation of the
environment from https://github.com/mimoralea/gym-walk. Just like with \ref bitrl_example_gymnasium_part_1, you will need
an instance of the  <a href="https://github.com/pockerman/bitrl-rest-api">bitsim-envs-api</a> server running in order 
to actually use the environment. Other than that there isn't much to say for this example.
Below is the driver code.

\include gym_walk/gym_walk.cpp
