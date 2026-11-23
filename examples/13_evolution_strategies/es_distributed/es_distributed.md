\page bitrl_example_es_distributed Evolution strategies over several ranks

\ref bitrl_example_es_cart_pole_swing_up learns to swing a pole up with an evolution strategy, on
the threads of one process. Here the episodes of every generation are shared out among several
*ranks*: threads, processes or machines. The ranks exchange only the returns of their episodes,
not the weights of the network or its gradients, and every rank ends with exactly the parameters
of a run on one rank.

## What you need

Nothing beyond bitrl. The ranks meet through `c10d::TCPStore`, the key-value store that comes with
libtorch, as in \ref bitrl_example_ppo_distributed. Ranks on other machines need to reach rank 0's
port.

## Only the returns travel

A generation of \ref bitrl::rl::algos::es::EvolutionStrategy "EvolutionStrategy" is \f$n\f$ pairs
of episodes, one at \f$\theta + \sigma\epsilon_k\f$ and one at \f$\theta - \sigma\epsilon_k\f$.
With ranks, rank \f$r\f$ plays the pairs \f$k\f$ with \f$k \bmod W = r\f$:

@code{.cpp}
TCPStoreCommunicator communicator(host, port, rank, n_ranks);   // rank 0 runs the store
EvolutionStrategy es(config, initial_parameters);              // alike on every rank
for (uint_t g = 0; g < generations; ++g)
    es.step(fitness, communicator);                            // this rank's pairs, then the others' returns
@endcode

The noise \f$\epsilon_k\f$ of pair \f$k\f$ in generation \f$g\f$ is drawn from the seed
`task_seed(task_seed(seed, g), k)`, so every rank can draw every direction itself. Once it has the
returns of all the pairs, every rank computes the same gradient, adding the pairs in the same order,
and takes the same Adam step. That keeps the ranks alike without ever sending a parameter, and the
run is the run on one rank, to the last bit.

The noise is drawn with \ref bitrl::rl::algos::es::PortableNormal "PortableNormal", the Box-Muller
transform of `std::mt19937_64`, rather than `std::normal_distribution`: the C++ standard fixes the
numbers of `std::mt19937_64` but leaves the normal distribution to the library, and ranks on
machines with different libraries would otherwise draw different noise.

A rank sends the returns of its own pairs, 16 numbers here, 128 bytes, whatever the size of the
network. \ref bitrl_example_ppo_distributed sends every rank's gradients instead, 4 bytes per
weight: a network of a million weights would send 4 MB per rank and update, and ES would still send
128 bytes. That is why evolution strategies scale to thousands of cores.

## Running the example

@code{.sh}
./es_distributed                                     # one rank, then 4 ranks as threads
for r in 0 1 2 3; do ./es_distributed --rank $r --ranks 4 & done; wait   # 4 processes
@endcode

With `--rank`, each process is one rank. Rank 0 runs the store at `--host` and `--port` (127.0.0.1
and 29541 by default), and the others connect to it, from this machine or another. Every rank plays
its episodes on one thread, so that the times compare ranks, not threads.

## Output

@code
Swinging the pole up: 300 generations of 64 episodes

1. One rank, on one thread
   8.67 s; the return of the mean policy 288.9

2. 4 ranks, each on one thread, every rank 8 of the 32 pairs
   2.99 s, a speed-up of 2.90; the return of the mean policy 288.9
   every rank ended with the parameters of the one rank, to the last bit: true (checksum 91f1b53333f47f40)
   every generation each rank sends the returns of its own pairs, at most 16 numbers, 128 bytes,
   and receives the others': 64 returns in all
   a gradient of this network would be 452 bytes; of a network of a million weights, 4 MB:
   the returns stay 128 bytes, however large the network
@endcode

Run as four processes, every rank printed checksum `91f1b53333f47f40` too, in 2.8 to 3.1 s.

Four ranks train about 3 times faster than one, not 4 times: every generation waits for the slowest
rank, and the ranks share the cores of this laptop with each other. The speed-up varies from run to
run, between about 2.5 and 3.4. On separate machines, each rank has its cores to itself.

## Things to try

1. Run 3 ranks. The 32 pairs no longer share out evenly: does the checksum still match?
2. Run two ranks on two machines: `--host` is rank 0's address on both.
3. Give the network 64 hidden units instead of 16. How much more does a generation cost, and how
   much more does a rank send?

## The code

\include es_distributed.cpp
