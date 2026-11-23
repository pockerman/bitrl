# Deep reinforcement learning. DQN uses bitrl's grid world; the others need
# the bitrl-rest-api server, except ppo_parallel_rollouts and ppo_distributed, which use bitrl's own CartPole
bitrl_add_example(dqn)
bitrl_add_example(dqn_experience_replay)
bitrl_add_example(reinforce)
bitrl_add_example(reinforce_baseline)
bitrl_add_example(a2c)
bitrl_add_example(ppo_cartpole)
bitrl_add_example(ppo_lunar_lander)
bitrl_add_example(ppo_parallel_rollouts SMOKE_TEST SMOKE_ARGS --episodes 64)
bitrl_add_example(ppo_distributed SMOKE_TEST SMOKE_ARGS --episodes 64)
