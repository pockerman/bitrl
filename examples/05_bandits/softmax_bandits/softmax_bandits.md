\page bitrl_example_softmax_bandits Multi-armed bandits with a soft-max policy

\ref bitrl_example_multi_armed_bandits solved a multi-armed bandit with \f$\epsilon\f$-greedy and Thompson sampling. This
example solves a 10-armed bandit with a soft-max policy instead. It follows the example in
<a href="https://www.manning.com/books/deep-reinforcement-learning-in-action">Deep Reinforcement Learning in Action</a>.

## The bandit

Each of the 10 arms has a probability \f$p_i\f$, drawn uniformly from \f$[0, 1]\f$ at the start. Pulling arm
\f$i\f$ gives a reward of 1 plus one for each of 10 trials that succeeds with probability \f$p_i\f$, so its
expected reward is \f$1 + 10 p_i\f$. The agent does not know the \f$p_i\f$. It keeps, for every arm, how often
it pulled it and the mean reward it got from it.

## The soft-max policy

Always pulling the arm with the best mean so far never tries the others, and an arm that was unlucky at first is
never tried again. A soft-max policy picks every arm with a probability that grows with its mean reward
\f$Q_i\f$,

\f[
P(i) = \frac{e^{Q_i / \tau}}{\sum_{j} e^{Q_j / \tau}} ,
\f]

where the temperature \f$\tau\f$, 0.7 here, sets how strongly it favours the best arms. A large \f$\tau\f$ picks
the arms almost uniformly; a small one almost always picks the best. `bitrl::maths::softmax_vec` computes the
probabilities, and a `std::discrete_distribution` picks an arm from them.

## What it writes

The example runs 500 pulls. After each one it prints the mean reward of all the pulls so far, and at the end it
writes these means to `rewards.csv`, one per line, in the directory it runs from. In one run the mean
started at 9, the reward of the first pull, dropped to 8.2 by the tenth pull, and settled at about 8.7
after some 50 pulls.

## The driver code

\include softmax_bandits/softmax_bandits.cpp
