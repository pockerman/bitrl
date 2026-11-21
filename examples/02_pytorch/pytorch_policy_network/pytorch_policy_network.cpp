// A policy network with the PyTorch C++ API
//
// The policy gradient methods of bitrl, REINFORCE, A2C and PPO, all have a
// network whose outputs define a distribution over the actions. They sample
// an action from it, and change the network so that the log probability of
// the actions that did well goes up. This example does that on its own, on a
// contextual bandit that needs no environment server: in each of two
// situations one of three arms pays best, and the network learns which.

#include "bitrl/base/bitrl_types.h"
#include "bitrl/maths/optimization/optimizer_type.h"
#include "bitrl/maths/optimization/pytorch_optimizer_factory.h"
#include "bitrl/maths/statistics/distributions/torch_categorical.h"

#include <torch/torch.h>

#include <any>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>

namespace pytorch_policy_network
{

using bitrl::real_t;
using bitrl::torch_tensor_t;
using bitrl::uint_t;
using bitrl::maths::optim::OptimizerType;
using bitrl::maths::stats::TorchCategorical;

const uint_t N_SITUATIONS = 2;
const uint_t N_ARMS = 3;
const uint_t BATCH_SIZE = 64;
const uint_t N_UPDATES = 300;
const real_t LEARNING_RATE = 0.01;
const real_t REWARD_NOISE = 0.5;

// The mean reward of each arm in each situation: arm 2 is best in the first,
// arm 0 in the second
const real_t MEAN_REWARD[N_SITUATIONS][N_ARMS] = {{0.2, 0.5, 1.0}, {1.0, 0.3, 0.1}};

// The policy: the situation, one-hot, in; a logit for every arm out
struct PolicyImpl : torch::nn::Module
{
    PolicyImpl()
        : hidden(register_module("hidden", torch::nn::Linear(N_SITUATIONS, 16))),
          out(register_module("out", torch::nn::Linear(16, N_ARMS)))
    {
    }

    torch_tensor_t forward(torch_tensor_t situation)
    {
        return out->forward(torch::relu(hidden->forward(situation)));
    }

    torch::nn::Linear hidden;
    torch::nn::Linear out;
};
TORCH_MODULE(Policy);

// The probability the policy gives every arm in every situation
void print_probabilities(Policy &policy, const std::string &when)
{
    torch::NoGradGuard no_grad;
    const auto situations = torch::eye(N_SITUATIONS);
    const auto probs = torch::softmax(policy->forward(situations), -1);
    std::cout << when << "\n";
    for (uint_t s = 0; s < N_SITUATIONS; ++s)
    {
        std::cout << "  situation " << s << ":";
        for (uint_t a = 0; a < N_ARMS; ++a)
        {
            std::cout << "  arm " << a << " " << std::fixed << std::setprecision(3)
                      << probs[s][a].item<real_t>();
        }
        std::cout << "\n";
    }
}

} // namespace pytorch_policy_network

int main()
{
    using namespace pytorch_policy_network;
    torch::manual_seed(42);

    Policy policy;
    print_probabilities(policy, "Before training");

    // bitrl builds the optimizer from its type and options
    std::map<std::string, std::any> options{{"lr", std::any(LEARNING_RATE)}};
    namespace optim = bitrl::maths::optim::pytorch;
    const auto optimizer_options =
        optim::build_pytorch_optimizer_options(OptimizerType::ADAM, options);
    auto optimizer =
        optim::build_pytorch_optimizer(OptimizerType::ADAM, *policy, *optimizer_options);

    const auto means = torch::tensor({MEAN_REWARD[0][0], MEAN_REWARD[0][1], MEAN_REWARD[0][2],
                                      MEAN_REWARD[1][0], MEAN_REWARD[1][1], MEAN_REWARD[1][2]},
                                     torch::kFloat64)
                           .view({N_SITUATIONS, N_ARMS});

    for (uint_t update = 0; update < N_UPDATES; ++update)
    {
        // a batch of situations, and an arm for each, sampled from the policy
        const auto situation_index = torch::randint(N_SITUATIONS, {BATCH_SIZE}, torch::kLong);
        const auto situations = torch::one_hot(situation_index, N_SITUATIONS).to(torch::kFloat32);
        TorchCategorical policy_distribution(policy->forward(situations), /*from logits*/ true);
        const auto arms = policy_distribution.sample().detach();

        // the bandit pays a noisy reward
        const auto rewards = (means.index({situation_index, arms}) +
                              REWARD_NOISE * torch::randn({BATCH_SIZE}, torch::kFloat64))
                                 .to(torch::kFloat32);

        // REINFORCE: raise the log probability of the arms that paid more than
        // the mean of the batch, lower it for the others
        const auto advantage = rewards - rewards.mean();
        const auto loss = -(policy_distribution.log_prob(arms) * advantage).mean();

        optimizer->zero_grad();
        loss.backward();
        optimizer->step();

        if ((update + 1) % 100 == 0)
        {
            std::cout << "update " << update + 1 << ": mean reward " << std::fixed
                      << std::setprecision(3) << rewards.mean().item<real_t>() << ", entropy "
                      << policy_distribution.entropy().mean().item<real_t>() << "\n";
        }
    }

    print_probabilities(policy, "After training");

    // the policy should have found the best arm of each situation
    torch::NoGradGuard no_grad;
    const auto probs = torch::softmax(policy->forward(torch::eye(N_SITUATIONS)), -1);
    const bool learned = probs[0][2].item<real_t>() > 0.9 && probs[1][0].item<real_t>() > 0.9;
    std::cout << (learned ? "The policy found the best arm of both situations"
                          : "The policy did not find the best arms")
              << std::endl;
    return learned ? 0 : 1;
}
