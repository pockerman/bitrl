\page bitrl_example_ppo_distributed PPO over several ranks

\ref bitrl_example_ppo_parallel_rollouts learns from several episodes at once in one thread. Here
the episodes of an update are shared out among several *ranks*: threads, processes, or machines.
Every rank plays its own episodes and computes the gradients of its own. The ranks then replace
their gradients with the mean over all of them, and take the same optimiser step. This is the
data parallelism of PyTorch's DistributedDataParallel, for bitrl's PPO.

## What you need

Nothing beyond bitrl. The ranks meet through `c10d::TCPStore`, the key-value store that comes
with libtorch. Ranks on other machines need to reach rank 0's port.

## How it works

\ref bitrl::rl::algos::pg::DistributedPPOSolver "DistributedPPOSolver" is
\ref bitrl::rl::algos::pg::ParallelPPOSolver "ParallelPPOSolver" with two changes:

- before training, every rank takes the networks of rank 0, so the ranks start alike;
- after every backward pass, before the optimisers step, the gradients of both networks are
  replaced with their mean over the ranks.

Every rank gets exactly the same mean, to the last bit, because the ranks add their gradients in
the same order, the order of the ranks. So every rank's Adam takes the same step, and the ranks
stay alike: they are one network, trained on the episodes of all of them. The example checks
that with a checksum of every rank's networks.

The ranks exchange numbers through a \ref bitrl::rl::distributed::Communicator "Communicator".
\ref bitrl::rl::distributed::TCPStoreCommunicator "TCPStoreCommunicator" moves them through
libtorch's TCPStore, which rank 0 runs:

@code{.cpp}
TCPStoreCommunicator communicator(host, port, rank, n_ranks);  // rank 0 runs the store
ParallelEnvs<CartPole> envs(n_copies);
envs.make("v1", {{"max_steps", uint_t(500)}}, {{"seed", task_seed(SEED, rank)}}); // per rank
DistributedPPOSolver<CartPole, Actor, Critic> solver(config, actor, critic,
                                                     actor_optimizer, critic_optimizer,
                                                     communicator);
solver.actions_before_training_begins(envs);                   // rank 0's networks
for (uint_t round = 0; round < n_rounds; ++round)
    solver.on_training_episode(envs, round);
@endcode

Every gradient goes through the store, so each rank receives the gradients of every rank. For
the networks of RL, a few thousand weights, that takes a fraction of a millisecond. Networks of
millions of weights want a ring all-reduce, such as Gloo's or MPI's, behind the same
`Communicator`. libtorch has Gloo compiled in, but does not ship its headers.

Each rank normalises the advantages of its own episodes. The actor samples with a generator of
its own, so ranks that are threads of one process do not share libtorch's generator.

## Running the example

@code{.sh}
./ppo_distributed                        # 4 ranks as threads, against one thread
for r in 0 1 2 3; do ./ppo_distributed --rank $r --ranks 4 & done; wait   # 4 processes
@endcode

With `--rank`, each process is one rank. Rank 0 runs the store at `--host` and `--port`
(127.0.0.1 and 29531 by default), and the others connect to it, from this machine or another.

## Output

@code
1. 800 episodes of CartPole, 8 per update
                                         time   last 10 %  greedy return
  ParallelPPOSolver, 1 thread           5.5 s       483.0          500.0
  DistributedPPOSolver, 4 ranks of 2    4.7 s       488.1          481.3

  every rank ended with the same networks: true (checksum 1784bf9ee21b0fb)

2. 1600 episodes of CartPole, 32 per update
                                         time   last 10 %  greedy return
  ParallelPPOSolver, 1 thread           6.6 s       478.0          497.0
  DistributedPPOSolver, 4 ranks of 8    4.0 s       490.4          500.0

  every rank ended with the same networks: true (checksum daa743e73905aec8)
@endcode

Run as four processes, the four ranks ended with checksum `1784bf9ee21b0fb` too: the same
networks as the four threads, to the last bit.

Sharing out the episodes saves little with 8 episodes per update. Most of a round is spent
playing the episodes, a step at a time, and a call of the policy costs about the same for the 2
states of a rank as for the 8 states of one thread. What the ranks share is the update. With 32
episodes per update the update takes longer, and 4 ranks train in less time. On one machine,
ParallelPPOSolver with its copies on threads does much the same. Ranks pay off when
the episodes outgrow one machine, or when every rank has an expensive simulation of its own.

The two solvers don't learn the same thing: the ranks normalise their advantages apart, and play
other episodes. Both learn to keep the pole up.

## Things to try

1. Run 8 ranks of 1 copy, and 2 ranks of 4. How do the times change?
2. Run two ranks on two machines: `--host` is rank 0's address on both.
3. Give one rank's copies a different `max_steps`. Do the ranks still end with the same networks?

## The code

\include ppo_distributed.cpp
