# Running experiments: logging them, and keeping what they learn

# export_policy and log_experiment train on a cliff walk written in C++,
# in common/, so they need no server
bitrl_add_example(export_policy SMOKE_TEST)
TARGET_INCLUDE_DIRECTORIES(export_policy PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/common)
bitrl_add_example(log_experiment SMOKE_TEST)
TARGET_INCLUDE_DIRECTORIES(log_experiment PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/common)

# tensorboard needs the bitrl-rest-api server, which runs Tensorboard
bitrl_add_example(tensorboard FILES config.json)
