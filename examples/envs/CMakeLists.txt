# The Gymnasium examples need the bitrl-rest-api server, and tensorboard a Tensorboard server
bitrl_add_example(gymnasium_part_1)
bitrl_add_example(gym_walk)
bitrl_add_example(gymnasium_part_2)
bitrl_add_example(gymnasium_part_3)
bitrl_add_example(acrobot_vector_env)
bitrl_add_example(tensorboard FILES config.json)
bitrl_add_example(grid_world SMOKE_TEST)
bitrl_add_example(connect2 SMOKE_TEST)
