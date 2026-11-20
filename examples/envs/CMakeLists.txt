# The Gymnasium examples need the bitrl-rest-api server, and tensorboard a Tensorboard server
cuberl_add_example(gymnasium_part_1)
cuberl_add_example(gym_walk)
cuberl_add_example(gymnasium_part_2)
cuberl_add_example(gymnasium_part_3)
cuberl_add_example(acrobot_vector_env)
cuberl_add_example(tensorboard FILES config.json)
cuberl_add_example(grid_world SMOKE_TEST)
cuberl_add_example(connect2 SMOKE_TEST)
