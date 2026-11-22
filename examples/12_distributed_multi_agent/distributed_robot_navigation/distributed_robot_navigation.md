\page bitrl_example_distributed_robot_navigation Robot controllers in processes of their own

The decentralised robots of \ref bitrl_example_multi_robot_navigation cross a circle to the
opposite point, each with its own controller. Here every controller runs in a participant of its
own, as it would on the computer of each robot. The robots drive exactly as they do when every
controller is in one process.

## What you need

Nothing beyond bitrl for the threads of one process. For processes, bitsim built with MQTT and a
broker such as mosquitto; see bitsim's install page.

## How it works

Every participant holds a copy of \ref bitrl::envs::multi_agent::MultiRobotNavigation
"MultiRobotNavigation", made with the same options and seed, and wraps it in a
\ref bitrl::envs::multi_agent::DistributedAgentEnv "DistributedAgentEnv" for its robot. At every
step the controller reads its own robot's observation and chooses a linear and an angular
velocity. `step()` sends these two numbers to the others, waits for theirs, and steps the copy
with all of them:

@code{.cpp}
auto copy = make_env(n_robots);
DistributedAgentEnv<MultiRobotNavigation> env(copy, robot, transport);
const auto me = controller(copy, robot);
run_agent_episode(env, me);
@endcode

\ref bitrl::rl::multi_agent::run_agent_episode "run_agent_episode" plays an episode with the
action of one agent, as \ref bitrl::rl::multi_agent::run_episode "run_episode" does with a team.
How an action travels is up to \ref bitrl::envs::multi_agent::AgentValues "AgentValues": numbers
and enums as one number, `std::array` and `std::vector` as their elements. Specialise it for
other actions.

The copies need the same joint actions to stay alike, so the participants run in lockstep. A
participant cannot run in wall-clock mode, as a real robot would, because its copy would then
move the robots with actions that came late. A real robot belongs in bitsim's distributed
simulation, which sends states, not actions; see bitsim's hardware-in-the-loop example.

## Running the example

@code{.sh}
./distributed_robot_navigation                  # in one process, then a thread per robot
for k in 0 1 2 3 4 5; do ./distributed_robot_navigation --transport mqtt --robot $k & done; wait
@endcode

The second line runs every robot in a process of its own, meeting on the broker at
`tcp://localhost:1883` (`--broker` for another, `--robots` for another number). Every process
writes the paths of all robots, as its copy saw them, to `trajectories_<robot>.csv`.

## Output

@code
1. Every controller in one process
  every robot arrived after 11.1 s of simulated time; 0 moves blocked; closest centres 0.55 m
  in 0.0002 s

2. A participant per robot
  every robot arrived after 11.1 s of simulated time; 0 moves blocked; closest centres 0.55 m
  6 threads in 0.0139 s, 125.6 microseconds per step
  every copy moved every robot as the one process did: true
@endcode

Run as six MQTT processes, with a broker that has `set_tcp_nodelay true`, the crossing took
0.12 to 0.22 s, and the six `trajectories_<robot>.csv` were the same, byte for byte.

## Things to try

1. Run 12 robots, as threads and as processes. How does the time per step grow?
2. Give the controller of one robot a bigger safety distance (`SENSE`), in its process only. Does
   anything go wrong, and why not? What if you change `"circle_radius"` in one process instead?
3. Make the actions of the controllers noisy, with a generator seeded differently in every
   process. Do the copies stay alike?

## The code

\include distributed_robot_navigation.cpp
