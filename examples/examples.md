\page bitrl_examples Examples

The examples of bitrl, in the order they build on each other: the basic ideas of reinforcement
learning, the PyTorch C++ API that the deep methods use, running experiments, the environments,
and then the algorithms, from bandits to deep reinforcement learning. Most of the environments are
Gymnasium's, which run on the <a href="https://github.com/pockerman/bitrl-rest-api">bitrl-rest-api</a>
server, so start it before you run the examples that use them.

## The basics

Estimating with samples, returns and Markov chains, which the algorithms below are built on.

- \ref bitrl_example_monte_carlo_integration
- \ref bitrl_example_importance_sampling
- \ref bitrl_example_discounted_returns
- \ref bitrl_example_markov_chain

## The PyTorch C++ API

bitrl's deep reinforcement learning uses libtorch, PyTorch's C++ API.

- \ref bitrl_example_pytorch_tensors
- \ref bitrl_example_pytorch_linear_regression
- \ref bitrl_example_pytorch_custom_model
- \ref bitrl_example_pytorch_policy_network
- \ref bitrl_example_cuda_vector_addition

## Experiments

- \ref bitrl_example_tensorboard

## Environments

- \ref bitrl_example_grid_world
- \ref bitrl_example_connect2
- \ref bitrl_example_gymnasium_part_1
- \ref bitrl_example_gymnasium_part_2
- \ref bitrl_example_gymnasium_part_3
- \ref bitrl_example_gym_walk
- \ref bitrl_example_acrobot_vector_env

## Multi-armed bandits

- \ref bitrl_example_multi_armed_bandits
- \ref bitrl_example_softmax_bandits

## Dynamic programming

- \ref bitrl_example_iterative_policy_evaluation
- \ref bitrl_example_policy_iteration
- \ref bitrl_example_value_iteration

## Monte Carlo methods

- \ref bitrl_example_first_visit_monte_carlo

## Temporal difference learning

- \ref bitrl_example_sarsa
- \ref bitrl_example_q_learning
- \ref bitrl_example_expected_sarsa

## Deep reinforcement learning

- \ref bitrl_example_dqn
- \ref bitrl_example_dqn_experience_replay
- \ref bitrl_example_reinforce
- \ref bitrl_example_reinforce_baseline
- \ref bitrl_example_a2c
- \ref bitrl_example_ppo_lunar_lander

## Robotics

bitsim's robots, simulated with Chrono. These need bitsim built with Chrono.

- \ref bitrl_example_chrono_smc_system
- \ref bitrl_example_chrono_rl_env
- \ref bitrl_example_mcts_line_following
