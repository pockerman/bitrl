\page bitrl_example_9 bitrl Example 9: Discounted returns

The return of an episode is the sum of the rewards the agent collects in it. Most algorithms discount the
rewards that come later by a factor \f$0 \le \gamma \le 1\f$ per step, so that a reward \f$k\f$ steps away is
worth \f$\gamma^k\f$ of a reward now. This example uses the two functions in `bitrl/rl/algorithms/utils.h` that
compute it.

## The return of a whole episode

`calculate_discounted_return(rewards, gamma)` gives the discounted return from the start of the episode,

\f[
G = \sum_{t=0}^{T} \gamma^{t} r_t .
\f]

With \f$\gamma = 1\f$ nothing is discounted and \f$G\f$ is the plain sum. The example checks that for ten rewards
of 1 the return is 10.

## The return from every step

Algorithms such as REINFORCE, see \ref bitrl_example_19, need the return from every step of the episode to its
end,

\f[
G_t = \sum_{k=t}^{T} \gamma^{k-t} r_k = r_t + \gamma G_{t+1} ,
\f]

which `calculate_step_discounted_return(rewards, gamma)` computes. For the rewards 0, 1, 2, 3, 4 and
\f$\gamma = 0.99\f$, the last step's return is its reward, 4, the one before it is \f$3 + 0.99 \cdot 4 = 6.96\f$,
and so on back to the first step.

## Output

The example prints each value next to the one worked out by hand:

@code{.sh}
Discounted reward: 10 expected: 10
step_discounted_reward size :5
Reward: 9.70348 expected: 9.70348
Reward: 9.8015 expected: 9.8015
Reward: 8.8904 expected: 8.8904
Reward: 6.96 expected: 6.96
Reward: 4 expected: 4
@endcode

## The driver code

\include example_9/example_9.cpp
