# Several agents in one environment: independent learners in a grid, and
# decentralised controllers of differential drive robots. Both run their
# independent trainings and simulations in parallel with bitsim's thread pool.
bitrl_add_example(multi_agent_grid_world SMOKE_TEST SMOKE_ARGS --episodes 500 --seeds 4)
bitrl_add_example(multi_robot_navigation SMOKE_TEST SMOKE_ARGS --robots 4)
