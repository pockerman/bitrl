// Proximal policy optimization on CartPole
//
// PPO is an actor-critic method. The actor, the policy, is a network whose
// outputs are the logits of a categorical distribution over CartPole's two
// actions, push left and push right; the critic is a network that estimates
// the value of a state. PPOSolver plays an episode, works out how much better
// every action did than the critic expected, and changes the policy towards
// the better actions, but no more than its clipped objective allows.
//
// After training the example plays one episode with the greedy policy and
// writes its actions to episode.csv, which examples/tools/episode_gif.py turns
// into a GIF.
//
// Needs the bitrl-rest-api server at SERVER_URL.

#include "bitrl/base/bitrl_types.h"
#include "bitrl/envs/gymnasium/classic_control/cart_pole_env.h"
#include "bitrl/maths/optimization/optimizer_type.h"
#include "bitrl/maths/optimization/pytorch_optimizer_factory.h"
#include "bitrl/maths/statistics/distributions/torch_categorical.h"
#include "bitrl/network/rest_rl_env_client.h"
#include "bitrl/rl/algorithms/pg/ppo.h"
#include "bitrl/rl/algorithms/pg/ppo_config.h"
#include "bitrl/rl/trainers/rl_serial_agent_trainer.h"
#include "bitsim/utils/io/csv_file_writer.h"

#include <torch/torch.h>

#include <any>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace ppo_cartpole
{

using bitrl::real_t;
using bitrl::torch_tensor_t;
using bitrl::uint_t;
using bitrl::maths::stats::TorchCategorical;
typedef bitrl::envs::gymnasium::CartPole env_type;

const std::string SERVER_URL = "http://0.0.0.0:8001/api";
const uint_t SEED = 42;
const uint_t N_EPISODES = 500;
const uint_t MAX_STEPS = 500;
const real_t LEARNING_RATE = 0.001;

torch_tensor_t to_tensor(const std::vector<real_t> &state)
{
    return torch::tensor(state, torch::dtype(torch::kFloat32));
}

// The actor: the 4 numbers of the state in, the logits of the 2 actions out
struct ActorImpl : torch::nn::Module
{
    ActorImpl()
        : fc1(register_module("fc1", torch::nn::Linear(4, 64))),
          fc2(register_module("fc2", torch::nn::Linear(64, 64))),
          logits(register_module("logits", torch::nn::Linear(64, 2)))
    {
    }

    torch_tensor_t forward(torch_tensor_t states)
    {
        return logits->forward(torch::tanh(fc2->forward(torch::tanh(fc1->forward(states)))));
    }

    // a sampled action and its log probability, for PPOSolver
    std::tuple<uint_t, torch_tensor_t> act(const std::vector<real_t> &state)
    {
        TorchCategorical distribution(forward(to_tensor(state)), /*from logits*/ true);
        const auto action = distribution.sample();
        return {static_cast<uint_t>(action.item<int64_t>()),
                distribution.log_prob(action).detach()};
    }

    // the log probabilities of the actions taken in the states, the entropy
    // of the policy, and the logits, for PPOSolver's update
    std::tuple<torch_tensor_t, torch_tensor_t, torch_tensor_t> evaluate(torch_tensor_t states,
                                                                        torch_tensor_t actions)
    {
        const auto out = forward(states);
        TorchCategorical distribution(out, /*from logits*/ true);
        return {distribution.log_prob(actions.view({-1})), distribution.entropy().mean(), out};
    }

    // the action the policy rates best, to play after training
    uint_t greedy(const std::vector<real_t> &state)
    {
        torch::NoGradGuard no_grad;
        return static_cast<uint_t>(forward(to_tensor(state)).argmax().item<int64_t>());
    }

    torch::nn::Linear fc1;
    torch::nn::Linear fc2;
    torch::nn::Linear logits;
};
TORCH_MODULE(Actor);

// The critic: the state in, its value out
struct CriticImpl : torch::nn::Module
{
    CriticImpl()
        : fc1(register_module("fc1", torch::nn::Linear(4, 64))),
          fc2(register_module("fc2", torch::nn::Linear(64, 64))),
          value(register_module("value", torch::nn::Linear(64, 1)))
    {
    }

    torch_tensor_t forward(torch_tensor_t states)
    {
        return value->forward(torch::tanh(fc2->forward(torch::tanh(fc1->forward(states)))));
    }

    torch_tensor_t evaluate(const std::vector<real_t> &state)
    {
        torch::NoGradGuard no_grad;
        return forward(to_tensor(state));
    }

    torch::nn::Linear fc1;
    torch::nn::Linear fc2;
    torch::nn::Linear value;
};
TORCH_MODULE(Critic);

typedef bitrl::rl::algos::pg::PPOSolver<env_type, Actor, Critic> solver_type;

} // namespace ppo_cartpole

int main()
{
    using namespace ppo_cartpole;
    namespace optim = bitrl::maths::optim;

    try
    {
        torch::manual_seed(SEED);
        bitrl::network::RESTRLEnvClient server(SERVER_URL, false);
        env_type env(server);
        std::unordered_map<std::string, std::any> options;
        std::unordered_map<std::string, std::any> reset_options{{"seed", std::any(SEED)}};
        env.make("v1", options, reset_options);

        bitrl::rl::algos::pg::PPOConfig config;
        config.n_episodes = N_EPISODES;
        config.max_itrs_per_episode = MAX_STEPS;
        config.gamma = 0.99;
        config.clip_epsilon = 0.2;
        config.max_passes_over_batch = 4;
        config.normalize_advantages = true;

        Actor actor;
        Critic critic;
        std::map<std::string, std::any> opt_options{{"lr", std::any(LEARNING_RATE)}};
        const auto options_adam = optim::pytorch::build_pytorch_optimizer_options(
            optim::OptimizerType::ADAM, opt_options);
        auto actor_optimizer = optim::pytorch::build_pytorch_optimizer(optim::OptimizerType::ADAM,
                                                                       *actor, *options_adam);
        auto critic_optimizer = optim::pytorch::build_pytorch_optimizer(optim::OptimizerType::ADAM,
                                                                        *critic, *options_adam);

        solver_type solver(config, actor, critic, actor_optimizer, critic_optimizer);
        bitrl::rl::RLSerialTrainerConfig trainer_config{50, N_EPISODES, 1.0e-8};
        bitrl::rl::RLSerialAgentTrainer<env_type, solver_type> trainer(trainer_config, solver);
        trainer.train(env);

        // the return of every training episode
        const auto &rewards = solver.get_monitor().rewards;
        bitsim::utils::io::CSVWriter rewards_file("rewards.csv", ',');
        rewards_file.open();
        rewards_file.write_column_names({"episode", "return"});
        for (uint_t e = 0; e < rewards.size(); ++e)
        {
            rewards_file.write_row(std::make_tuple(e, rewards[e]));
        }
        rewards_file.close();

        // one episode with the greedy policy, its actions for the GIF
        std::ofstream episode("episode.csv");
        episode << "# seed " << SEED << "\n";
        auto time_step = env.reset();
        real_t total = 0.0;
        for (uint_t step = 0; step < MAX_STEPS && !time_step.done(); ++step)
        {
            const auto action = actor->greedy(time_step.observation());
            episode << action << "\n";
            time_step = env.step(action);
            total += time_step.reward();
        }
        env.close();

        real_t last_50 = 0.0;
        for (auto i = rewards.size() - 50; i < rewards.size(); ++i)
        {
            last_50 += rewards[i] / 50.0;
        }
        std::cout << "Mean return of the last 50 training episodes: " << last_50 << "\n"
                  << "The greedy policy kept the pole up for " << total << " steps of " << MAX_STEPS
                  << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
