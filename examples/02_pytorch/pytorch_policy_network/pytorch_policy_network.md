\page bitrl_example_pytorch_policy_network A policy network

The policy gradient methods of bitrl, \ref bitrl_example_reinforce "REINFORCE",
\ref bitrl_example_a2c "A2C" and PPO, all rest on the same few steps with libtorch: a network whose outputs
define a distribution over the actions, an action sampled from it, and a change to the network that raises the
log probability of the actions that did well. This example does those steps on their own, on a problem that
needs no environment server, before the examples that use them on real environments.

## The problem

A contextual bandit: the agent is in one of two situations, chosen at random, and pulls one of three arms. Each
arm pays a reward with noise around a mean that depends on the situation:

| Situation | Arm 0 | Arm 1 | Arm 2 |
| --------- | ----- | ----- | ----- |
| 0 | 0.2 | 0.5 | 1.0 |
| 1 | 1.0 | 0.3 | 0.1 |

The best arm is different in the two situations, so the policy has to depend on the situation, which is what a
network is for.

## The network and the distribution

`Policy` takes the situation, one-hot, and gives a logit for every arm, through a hidden layer of 16 units.
\ref bitrl::maths::stats::TorchCategorical "TorchCategorical", built from the logits, is the distribution over the
arms, \f$\pi_\theta(\alpha | s)\f$, the soft-max of the logits. It samples the arms and gives their log
probabilities and its entropy. The optimizer, Adam with a learning rate of 0.01, comes from bitrl's
`build_pytorch_optimizer`, which builds a libtorch optimizer from its type and options.

## Learning

Every update samples 64 situations, an arm for each from the policy, and the rewards of the arms. The loss is the
REINFORCE loss,

\f[
L(\theta) = - \frac{1}{64} \sum_i \log \pi_\theta(\alpha_i | s_i) \left( r_i - \bar{r} \right),
\f]

where \f$\bar{r}\f$ is the mean reward of the batch. Its gradient raises the log probability of the arms that paid
more than the mean and lowers it for the others; subtracting the mean, a baseline, makes the gradient vary less
without changing its expected direction. The arms are sampled, so the loss does not differentiate through the
sampling, only through the log probabilities.

## The results

The policy starts close to uniform and after 300 updates picks the best arm of each situation almost always:

@code{.sh}
Before training
  situation 0:  arm 0 0.472  arm 1 0.340  arm 2 0.188
  situation 1:  arm 0 0.501  arm 1 0.263  arm 2 0.236
update 100: mean reward 0.976, entropy 0.019
update 200: mean reward 1.000, entropy 0.008
update 300: mean reward 1.020, entropy 0.006
After training
  situation 0:  arm 0 0.000  arm 1 0.000  arm 2 0.999
  situation 1:  arm 0 0.999  arm 1 0.000  arm 2 0.001
The policy found the best arm of both situations
@endcode

The entropy of the policy falls as it stops exploring. Here that is what we want, but on harder problems a policy
that stops exploring this early can settle on a poor action, which is why A2C and PPO can add the entropy to the
loss to keep it up. The example exits with an error if the best arms are not found, so it runs as a test too.

## The driver code

\include pytorch_policy_network/pytorch_policy_network.cpp
