# Every agent in a participant of its own: a thread, a process or a machine,
# meeting the others through bitsim's distributed transports. Every participant
# holds a copy of the environment, and only the actions travel.
bitrl_add_example(distributed_independent_learners SMOKE_TEST SMOKE_ARGS --episodes 300)
bitrl_add_example(distributed_robot_navigation SMOKE_TEST SMOKE_ARGS --robots 4)
