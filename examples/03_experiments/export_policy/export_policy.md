\page bitrl_example_export_policy Exporting a policy

Training is usually done once, and what it learned used later: by another program, on another machine, in a
robot. This example trains Q-learning and keeps the policy it found in two forms, a table of actions in a CSV file
and a network saved with libtorch, and loads each back into a new policy that then plays an episode.

It trains on the cliff walk of Sutton and Barto, example 6.6, a 4 x 12 grid written in C++ in
`examples/03_experiments/common/cliff_walk.h`, so it needs no environment server. The agent starts in the bottom
left corner and the episode ends in the bottom right one; every step costs 1, and the cells between them along the
bottom are a cliff that costs 100 and ends the episode.

## A table of actions

\ref bitrl::rl::algos::td::QLearningSolver "QLearningSolver" trains with an \f$\epsilon\f$-greedy policy,
\f$\epsilon = 0.1\f$, for 500 episodes, through
\ref bitrl::rl::RLSerialAgentTrainer "RLSerialAgentTrainer". `build_policy()` then gives the greedy policy of its
table, a \ref bitrl::rl::policies::MaxTabularPolicy "MaxTabularPolicy", which saves itself as a CSV file of states
and actions:

@code{.cpp}
const auto trained = solver.build_policy();
trained.save("policy.csv");

MaxTabularPolicy loaded;
loaded.load("policy.csv");
auto action = loaded.on_state(state);
@endcode

`load` reads the file back, and throws if it cannot be read or a row is not a state and an action. The example
checks that the loaded policy gives the same action in every state, and plays an episode with it.

## A network

A table works when there are few states. With more, a policy is a network, and libtorch saves and loads it with
`torch::save` and `torch::load`. To have a network to save, the example trains one to give the action of the table
in every state: the state, one-hot, in, a score for every action out, with a hidden layer of 64 units, trained with
the cross entropy between its scores and the table's actions. Then

@code{.cpp}
torch::save(network, "policy_network.pt");

PolicyNetwork loaded_network;
torch::load(loaded_network, "policy_network.pt");
@endcode

The loaded network has to be the same type as the saved one: the file holds its parameters, not its layers. The
example checks that the loaded network takes the table's action in every state, and plays an episode with it.

## The results

@code{.sh}
Q-learning's greedy policy, saved to policy.csv:
............
............
************
SCCCCCCCCCCG
Loaded 48 states from policy.csv, 0 different from the trained policy; it scores -13

The network saved to policy_network.pt and loaded again differs from the table in 0 states; it scores -13
@endcode

Q-learning learns the shortest path, along the edge of the cliff, 13 steps, and both loaded policies take it. The
example exits with an error if either loaded policy differs from the trained one, so it runs as a test too.

## The driver code

\include export_policy/export_policy.cpp
