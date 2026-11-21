\page bitrl_example_expected_sarsa Expected SARSA on CliffWorld

This example trains an agent on <a href="https://gymnasium.farama.org/environments/toy_text/cliff_walking/">CliffWalking</a>
from Gymnasium with expected SARSA. It has the same settings as \ref bitrl_example_sarsa, which uses SARSA, so the two can be
compared. The environment is \ref bitrl::envs::gymnasium::CliffWorld "CliffWorld", and it runs on the
<a href="https://github.com/pockerman/bitrl-rest-api">bitrl-rest-api</a> server, which the example talks to over HTTP at
`http://0.0.0.0:8001/api`, so start the server first.

## Expected SARSA

The agent walks on a 4 x 12 grid from the bottom left corner to the bottom right one. The cells between them along the
bottom are a cliff: stepping onto it costs 100 and sends the agent back to the start. Every other step costs 1.

SARSA moves the value of the action it took towards the reward plus the value of the action it takes next,

\f[
Q(s_t, \alpha_t) \leftarrow Q(s_t, \alpha_t) + \eta \left[ r_{t+1} + \gamma Q(s_{t+1}, \alpha_{t+1}) - Q(s_t, \alpha_t) \right] .
\f]

The next action is random some of the time, so the target is noisy. Expected SARSA replaces the value of that one action
by the value the policy expects at the next state, every action weighed with the probability the policy takes it:

\f[
Q(s_t, \alpha_t) \leftarrow Q(s_t, \alpha_t) + \eta \left[ r_{t+1} + \gamma \sum_{\alpha} \pi(\alpha | s_{t+1}) Q(s_{t+1}, \alpha) - Q(s_t, \alpha_t) \right] .
\f]

For the \f$\epsilon\f$-greedy policy the agent follows, with \f$n\f$ actions, \f$\pi(\alpha | s)\f$ is
\f$\epsilon / n\f$ for every action, plus \f$1 - \epsilon\f$ for the greedy one, so the expected value is
\f$\epsilon\f$ times the mean of the values at \f$s_{t+1}\f$ plus \f$1 - \epsilon\f$ times the largest.
\ref bitrl::rl::algos::td::ExpectedSarsaSolver "ExpectedSarsaSolver" computes it with the same greedy action as the
policy, the first of the largest values, and asks the policy for its \f$\epsilon\f$ every time, so a decaying
\f$\epsilon\f$ works too. After the step that ends an episode the target is the reward alone.

The target no longer depends on which action happens to come next, so the updates vary less than SARSA's, and expected
SARSA can learn with a larger learning rate. With \f$\epsilon = 0\f$ the policy is greedy, the expected value is the
largest value, and expected SARSA is Q-learning.

Like SARSA, expected SARSA learns the value of the policy it follows, which explores. A random step taken from next to
the cliff falls off it, so both learn to keep away from the cliff, while Q-learning, which learns the value of the
greedy policy, takes the shortest path along its edge. The unit test `test_td_cliff` shows this on a 3 x 5 cliff walk:
for every one of five seeds SARSA and expected SARSA go by the top row, and Q-learning along the edge.

## Settings and what it writes

The policy is \f$\epsilon\f$-greedy with a constant \f$\epsilon = 0.1\f$. The solver uses \f$\gamma = 1\f$ and
\f$\eta = 0.5\f$, and \ref bitrl::rl::RLSerialAgentTrainer "RLSerialAgentTrainer" runs 2000 episodes of at most 100
steps. In the directory it runs from, the example writes

| File | What |
| ---- | ---- |
| `expected_sarsa_cliff_walking_v0.csv` | the value of every state and action, a column per action |
| `reward_per_itr.csv` | the total reward of every training episode |
| `policy.csv` | the greedy policy, the action to take in each state |

## The driver code

\include expected_sarsa/expected_sarsa.cpp
