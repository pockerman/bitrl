// Exporting a policy after training
//
// Training is often done once, and the policy it found used later, by another
// program, on another machine or in a robot. This example trains Q-learning
// on the cliff walk and keeps what it learned in two forms:
//
// - the greedy policy of the table, as a CSV file of states and actions, which
//   MaxTabularPolicy saves and loads;
// - a small network trained to give the same actions, saved with libtorch's
//   torch::save, the form a policy takes when there are too many states for a
//   table.
//
// Each file is loaded back into a new policy, which then plays an episode.

#include "cliff_walk.h"

#include "bitrl/base/bitrl_types.h"
#include "bitrl/rl/algorithms/td/q_learning.h"
#include "bitrl/rl/policies/epsilon_greedy_policy.h"
#include "bitrl/rl/policies/max_tabular_policy.h"
#include "bitrl/rl/trainers/rl_serial_agent_trainer.h"

#include <torch/torch.h>

#include <iostream>
#include <string>

namespace export_policy
{

using bitrl::real_t;
using bitrl::torch_tensor_t;
using bitrl::uint_t;
using bitrl::rl::policies::EpsilonGreedyPolicy;
using bitrl::rl::policies::MaxTabularPolicy;
using examples::CliffWalk;

const std::string POLICY_FILE = "policy.csv";
const std::string NETWORK_FILE = "policy_network.pt";

// The network: the state, one-hot, in; a score for every action out
struct PolicyNetworkImpl : torch::nn::Module
{
    PolicyNetworkImpl()
        : hidden(register_module("hidden", torch::nn::Linear(CliffWalk().n_states(), 64))),
          out(register_module("out", torch::nn::Linear(64, CliffWalk().n_actions())))
    {
    }

    torch_tensor_t forward(torch_tensor_t states)
    {
        return out->forward(torch::relu(hidden->forward(states)));
    }

    // the action with the highest score in the given state
    uint_t act(uint_t state)
    {
        torch::NoGradGuard no_grad;
        const auto one_hot =
            torch::one_hot(torch::tensor({static_cast<int64_t>(state)}), CliffWalk().n_states())
                .to(torch::kFloat32);
        return forward(one_hot).argmax(-1).item<int64_t>();
    }

    torch::nn::Linear hidden;
    torch::nn::Linear out;
};
TORCH_MODULE(PolicyNetwork);

// The return of one episode with the given policy, at most 100 steps
template <typename Policy> real_t play(Policy &&policy)
{
    CliffWalk env;
    auto time_step = env.reset();
    real_t total = 0.0;
    for (uint_t i = 0; i < 100 && !time_step.done(); ++i)
    {
        time_step = env.step(policy(time_step.observation()));
        total += time_step.reward();
    }
    return total;
}

MaxTabularPolicy train_q_learning()
{
    CliffWalk env;
    bitrl::rl::algos::td::QLearningConfig config;
    config.n_episodes = 500;
    config.max_num_iterations_per_episode = 100;
    config.tolerance = 1.0e-8;
    config.gamma = 1.0;
    config.eta = 0.5;
    typedef bitrl::rl::algos::td::QLearningSolver<CliffWalk, EpsilonGreedyPolicy> solver_type;
    solver_type solver(config, EpsilonGreedyPolicy(0.1, 42));

    // report every 1000 episodes, so not while these 500 run
    bitrl::rl::RLSerialTrainerConfig trainer_config{1000, config.n_episodes, 1.0e-8};
    bitrl::rl::RLSerialAgentTrainer<CliffWalk, solver_type> trainer(trainer_config, solver);
    trainer.train(env);
    return solver.build_policy();
}

// Train the network to give the action of the table in every state
void imitate(PolicyNetwork &network, const MaxTabularPolicy &policy)
{
    const auto n_states = static_cast<int64_t>(policy.n_states());
    const auto states = torch::eye(n_states);
    std::vector<int64_t> actions(n_states);
    for (int64_t s = 0; s < n_states; ++s)
    {
        actions[s] = policy.on_state(s);
    }
    const auto targets = torch::tensor(actions);

    torch::optim::Adam optimizer(network->parameters(), torch::optim::AdamOptions(0.01));
    for (uint_t epoch = 0; epoch < 500; ++epoch)
    {
        const auto loss = torch::nn::functional::cross_entropy(network->forward(states), targets);
        optimizer.zero_grad();
        loss.backward();
        optimizer.step();
    }
}

} // namespace export_policy

int main()
{
    using namespace export_policy;
    torch::manual_seed(42);

    // Train, and save the greedy policy of the table
    const auto trained = train_q_learning();
    trained.save(POLICY_FILE);
    std::cout << "Q-learning's greedy policy, saved to " << POLICY_FILE << ":\n"
              << CliffWalk::draw([&](uint_t s) { return trained.on_state(s); });

    // Load it into a new policy, as another program would
    MaxTabularPolicy loaded;
    loaded.load(POLICY_FILE);
    uint_t differences = 0;
    for (uint_t s = 0; s < loaded.n_states(); ++s)
    {
        differences += loaded.on_state(s) != trained.on_state(s);
    }
    const real_t table_return = play([&](uint_t s) { return loaded.on_state(s); });
    std::cout << "Loaded " << loaded.n_states() << " states from " << POLICY_FILE << ", "
              << differences << " different from the trained policy; it scores " << table_return
              << "\n\n";

    // A network that gives the same actions, saved with torch::save
    PolicyNetwork network;
    imitate(network, loaded);
    torch::save(network, NETWORK_FILE);

    // Load it into a new network and play with it
    PolicyNetwork loaded_network;
    torch::load(loaded_network, NETWORK_FILE);
    uint_t network_differences = 0;
    for (uint_t s = 0; s < loaded.n_states(); ++s)
    {
        network_differences += loaded_network->act(s) != loaded.on_state(s);
    }
    const real_t network_return = play([&](uint_t s) { return loaded_network->act(s); });
    std::cout << "The network saved to " << NETWORK_FILE << " and loaded again differs from the "
              << "table in " << network_differences << " states; it scores " << network_return
              << "\n";

    const bool same = differences == 0 && network_differences == 0;
    return same ? 0 : 1;
}
