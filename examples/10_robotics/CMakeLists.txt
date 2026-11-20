# bitsim's robots, simulated with Chrono, as reinforcement learning
# environments and for planning
bitrl_add_example(chrono_smc_system REQUIRES CHRONO)
bitrl_add_example(chrono_rl_env REQUIRES CHRONO IRRLICHT)
bitrl_add_example(mcts_line_following REQUIRES CHRONO IRRLICHT OPENCV SMOKE_TEST SMOKE_ARGS --time-limit 10)
