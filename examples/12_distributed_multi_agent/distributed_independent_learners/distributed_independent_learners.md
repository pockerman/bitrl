\page bitrl_example_distributed_independent_learners Independent learners in processes of their own

The two agents of \ref bitrl_example_multi_agent_grid_world learn to share a corridor one cell
wide with independent Q-learning. Here every agent learns in a participant of its own: a thread,
a process, or a machine. Nothing else changes: the agents learn exactly the Q-tables they learn
in one process, to the last bit.

## What you need

Nothing beyond bitrl for the threads of one process. For processes, bitsim built with MQTT and a
broker such as mosquitto; see bitsim's install page.

## A copy of the world in every participant

There are two ways to split a multi-agent simulation. bitsim's distributed simulation gives every
participant its own robots, and the participants send each other where their robots are. That
suits a world that is expensive, or not repeatable, such as a Chrono simulation. In RL the
environment is often cheap next to the agents, and repeatable. Then every participant can hold a
copy of the whole environment, and only the actions need to travel.

\ref bitrl::envs::multi_agent::DistributedAgentEnv "DistributedAgentEnv" does that. It wraps the
participant's copy, made with the same options and seed as every other copy, and is the
environment of one agent:

@code{.cpp}
auto copy = make_env();                            // the same options and seed everywhere
QLearningAgent learner(agent, copy.n_states(), copy.n_actions(), config);
DistributedAgentEnv<MultiAgentGridWorld> env(copy, agent, transport);

AgentTrainer<DistributedAgentEnv<MultiAgentGridWorld>, QLearningAgent> trainer({n_episodes}, learner);
trainer.train(env);
@endcode

`env.step(action)` sends this agent's action, waits for the actions of the others at the same
step, with bitsim's \ref bitsim::distributed::Synchroniser "Synchroniser" in lockstep, and steps
the copy with the joint action. Every copy gets the same joint actions, so the copies stay alike.
To make sure, every action comes with a checksum of the observation it was chosen from. If it is
not the observation of that agent in this participant's copy, `step()` throws. That catches a
copy made with other options, reset with another seed, or one that is not deterministic.

The learner is a \ref bitrl::rl::multi_agent::QLearningAgent "QLearningAgent": one agent's
Q-table and epsilon-greedy policy, whose generator is seeded from the seed and the index of the
agent. \ref bitrl::rl::multi_agent::IndependentQLearning "IndependentQLearning" is a team of them,
so a QLearningAgent on its own learns what the same agent learns in the team.
\ref bitrl::rl::multi_agent::AgentTrainer "AgentTrainer" is the training loop of
\ref bitrl::rl::multi_agent::MultiAgentTrainer "MultiAgentTrainer" for one agent.

## Running the example

@code{.sh}
./distributed_independent_learners                   # in one process, then a thread per agent
./distributed_independent_learners --transport mqtt --agent 1 &
./distributed_independent_learners --transport mqtt --agent 0
@endcode

The second and third lines run every agent in a process of its own, meeting on the broker at
`tcp://localhost:1883` (`--broker` for another). Each process also trains the team in one
process, to tell whether it learned the same.

## Output

@code
1. The team in one process
  4000 episodes, 90324 steps, in 0.02 s
  the greedy team crossed in 15 steps, team return 1.70

2. A participant per agent
  2 threads, one per agent, in 7.52 s: 83.2 microseconds per step for the exchange
  the greedy team crossed in 15 steps, team return 1.70
  every agent learned the same Q-table, and every episode had the same returns, as in one process: true
@endcode

Over MQTT, with a broker that has `set_tcp_nodelay true`, the two processes took 11.7 s, 129
microseconds a step, and learned the same Q-tables. With mosquitto's default settings a step
waits a few milliseconds for the broker to bundle small packets; see the note on brokers in
bitsim's distributed simulation example.

The distributed training is much slower than the one in one process. Every step waits for a
message from the other agent, and a step of a tabular Q-learner takes well under a microsecond.
Splitting pays off when an agent is the expensive part, such as a neural network policy, or when
it has to run somewhere else, on a robot's computer for one. The time per step is the price of
the exchange, and it does not grow with what the agents compute.

## Things to try

1. Make one participant's copy with `GridObservation::OWN_POSITION`, and run the two processes.
   What does the first step say? Why does a different `"seed"` change nothing here?
2. Start the two processes ten seconds apart. What does the first one do in the meantime?
3. Give `DistributedAgentConfig::timeout` 1 s and stop one process half-way. What do you see in
   the other?

## The code

\include distributed_independent_learners.cpp
