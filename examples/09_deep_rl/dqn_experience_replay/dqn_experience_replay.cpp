/**
 * Use DQN on Gridworld
 *
 * */
#include "bitrl/base/bitrl_config.h"

#ifdef USE_PYTORCH

#include "bitrl/base/bitrl_types.h"
#include "bitrl/maths/optimization/optimizer_type.h"
#include "bitrl/maths/optimization/pytorch_optimizer_factory.h"
#include "bitrl/maths/vector_math.h"
#include "bitrl/rl/policies/epsilon_greedy_policy.h"
#include "bitrl/rl/trainers/rl_serial_agent_trainer.h"
#include "bitrl/utils/torch_adaptor.h"

#include "bitrl/base/bitrl_log.h"
#include "bitrl/envs/grid_world/grid_world_env.h"
#include "bitrl/utils/experience_buffer.h"
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

namespace rl_example_12
{

namespace F = torch::nn::functional;

using bitrl::float_t;
using bitrl::int_t;
using bitrl::real_t;
using bitrl::torch_tensor_t;
using bitrl::uint_t;

using bitrl::envs::grid_world::Gridworld;
using bitrl::rl::RLSerialAgentTrainer;
using bitrl::rl::RLSerialTrainerConfig;
using bitrl::rl::policies::EpsilonGreedyPolicy;
using bitrl::utils::ExperienceBuffer;
using bitsim::utils::io::CSVWriter;

const std::string EXPERIMENT_ID = "1";

constexpr uint_t l1 = 64;
constexpr uint_t l2 = 150;
constexpr uint_t l3 = 100;
constexpr uint_t l4 = 4;
constexpr uint_t SEED = 42;
constexpr uint_t BATCH_SIZE = 200;
constexpr uint_t EXPERIENCE_BUFFER_SIZE = 1000;
constexpr uint_t TOTAL_EPOCHS = 5000;
constexpr uint_t TOTAL_ITRS_PER_EPOCH = 50;
constexpr real_t GAMMA = 0.9;
constexpr real_t EPSILON = 0.3;
constexpr real_t LEARNING_RATE = 1.0e-3;

class QNetImpl : public torch::nn::Module
{
  public:
    QNetImpl();
    torch_tensor_t forward(torch_tensor_t);

  private:
    torch::nn::Linear fc1_;
    torch::nn::Linear fc2_;
    torch::nn::Linear fc3_;
};

QNetImpl::QNetImpl()
    : fc1_(torch::nn::Linear(l1, l2)), fc2_(torch::nn::Linear(l2, l3)),
      fc3_(torch::nn::Linear(l3, l4))
{
    register_module("fc1", fc1_);
    register_module("fc2", fc2_);
    register_module("fc3", fc3_);
}

torch_tensor_t QNetImpl::forward(torch_tensor_t x)
{

    // ReLU after the hidden layers only: the Q values are negative, every step
    // pays -1, and a ReLU on the output could not give them
    x = F::relu(fc1_->forward(x));
    x = F::relu(fc2_->forward(x));
    return fc3_->forward(x);
}

// create the model
TORCH_MODULE(QNet);

// 4x4 grid
typedef Gridworld<4> env_type;
typedef env_type::time_step_type time_step_type;

typedef std::tuple<std::vector<float_t>, uint_t, float_t, std::vector<float_t>, uint_t>
    experience_tuple_type;

typedef bitrl::utils::ExperienceBuffer<experience_tuple_type> experience_buffer_type;
typedef env_type::state_type state_type;

std::vector<float_t> flattened_observation(const state_type &state)
{

    std::vector<float_t> data;
    data.reserve(64);

    for (uint_t i = 0; i < state.size(); ++i)
    {
        for (uint_t j = 0; j < state[i].size(); ++j)
        {
            for (uint_t k = 0; k < state[i][j].size(); ++k)
            {
                data.push_back(state[i][j][k]);
            }
        }
    }

    return data;
}

template <typename T, uint_t index>
std::vector<T> get(const std::vector<experience_tuple_type> &experience)
{

    std::vector<T> result;
    result.reserve(experience.size());

    auto b = experience.begin();
    auto e = experience.end();

    for (; b != e; ++b)
    {
        auto item = *b;
        result.push_back(std::get<index>(item));
    }

    return result;
}

} // namespace rl_example_12

int main()
{

    using namespace rl_example_12;
    using namespace bitrl;

    try
    {

        BITRL_LOG_INFO << "Starting agent training...";
        BITRL_LOG_INFO << "Number of episodes to train: " << TOTAL_EPOCHS;

        // let's create a directory where we want to
        // store all the results from running a simulation
        std::filesystem::create_directories("experiments/" + EXPERIMENT_ID);

        // set the seed for PyTorch
        torch::manual_seed(SEED);

        BITRL_LOG_INFO << "Creating the environment...";

        // create a 4x4 grid
        auto env = env_type();

        // initialize the environment using random mode
        std::unordered_map<std::string, std::any> options;
        options["mode"] = std::any(bitrl::envs::grid_world::GridWorldInitType::RANDOM);

        std::unordered_map<std::string, std::any> reset_options;
        env.make("v0", options, reset_options);

        BITRL_LOG_INFO << "Done...";
        BITRL_LOG_INFO << "Environment name: " << env.name;
        BITRL_LOG_INFO << "Number of actions available: " << env.n_actions();
        BITRL_LOG_INFO << "Number of states available: " << env.n_states();

        // the network to train for the q values
        QNet qnet;

        auto optimizer_ptr = std::make_unique<torch::optim::Adam>(
            qnet->parameters(), torch::optim::AdamOptions(LEARNING_RATE));

        // we will use an epsilon-greedy policy
        EpsilonGreedyPolicy policy(EPSILON, SEED, rl::policies::EpsilonDecayOption::NONE);

        // the loss function to use
        auto loss_fn = torch::nn::MSELoss();

        // track the average loss per epoch
        std::vector<real_t> losses;
        losses.reserve(TOTAL_EPOCHS);

        experience_buffer_type experience_buffer(EXPERIENCE_BUFFER_SIZE);

        // hold random values
        std::vector<float_t> rand_vec(64, 0.0);
        float_t a = 0.0;
        float_t b = 1.0;

        // loop over the epochs
        for (uint_t epoch = 0; epoch < TOTAL_EPOCHS; ++epoch)
        {

            BITRL_LOG_INFO << "Starting epoch: " << epoch << std::endl;

            // for every new epoch we reset the environment
            auto time_step = env.reset();
            uint_t step_counter = 0;

            // the loss associated with the epoch
            std::vector<real_t> epoch_loss;
            epoch_loss.reserve(TOTAL_ITRS_PER_EPOCH);

            auto done = false;
            while (!done)
            {

                auto obs1 = flattened_observation(time_step.observation());

                rand_vec = bitrl::maths::randomize(rand_vec, a, b, 64);
                rand_vec = bitrl::maths::divide(rand_vec, 100.0);

                // randomize the flattened observation by adding
                // some noise
                obs1 = maths::add(obs1, rand_vec);
                auto torch_state_1 =
                    bitrl::utils::pytorch::TorchAdaptor::to_torch(obs1, DeviceType::CPU);

                // get the qvals
                auto qvals = qnet(torch_state_1);
                auto action_idx = policy(qvals, bitrl::torch_tensor_value_type<float_t>());

                BITRL_LOG_INFO << "Action selected: " << action_idx << std::endl;

                // step in the environment
                time_step = env.step(action_idx);
                auto reward = time_step.reward();
                auto step_finished =
                    time_step.done() ? static_cast<uint_t>(1) : static_cast<uint_t>(0);

                auto obs2 = flattened_observation(time_step.observation());
                rand_vec = bitrl::maths::randomize(rand_vec, a, b, 64);
                rand_vec = bitrl::maths::divide(rand_vec, 100.0);
                obs2 = maths::add(obs2, rand_vec);

                auto torch_state_2 =
                    bitrl::utils::pytorch::TorchAdaptor::to_torch(obs2, DeviceType::CPU);

                experience_tuple_type exp = {obs1, action_idx, reward, obs2, step_finished};

                // put the observation into the buffer
                experience_buffer.append(exp);

                // if we reach the batch size we will do
                // a backward propagation
                if (experience_buffer.size() >= BATCH_SIZE)
                {

                    std::vector<experience_tuple_type> batch_sample;
                    batch_sample.reserve(BATCH_SIZE);

                    // sample from the experience
                    experience_buffer.sample(BATCH_SIZE, batch_sample, SEED);

                    // stack the experiences
                    auto state_1_batch = get<std::vector<float_t>, 0>(batch_sample);
                    auto action_batch = get<uint_t, 1>(batch_sample);
                    auto state_2_batch = get<std::vector<float_t>, 3>(batch_sample);
                    auto reward_batch = get<float_t, 2>(batch_sample);
                    auto done_batch = get<uint_t, 4>(batch_sample);

                    auto state_1_batch_t = bitrl::utils::pytorch::TorchAdaptor::stack(
                        state_1_batch, bitrl::DeviceType::CPU);
                    auto q1 = qnet(state_1_batch_t);

                    // tell the model that we don't use grad here
                    qnet->eval();

                    auto state_2_batch_t = bitrl::utils::pytorch::TorchAdaptor::stack(
                        state_2_batch, bitrl::DeviceType::CPU);
                    auto q2 = qnet(state_2_batch_t);

                    // we are training again
                    qnet->train();

                    auto reward_batch_t = bitrl::utils::pytorch::TorchAdaptor::to_torch(
                        reward_batch, bitrl::DeviceType::CPU);
                    auto done_batch_t = bitrl::utils::pytorch::TorchAdaptor::to_torch(
                        done_batch, bitrl::DeviceType::CPU);
                    auto action_batch_t = bitrl::utils::pytorch::TorchAdaptor::to_torch(
                        action_batch, bitrl::DeviceType::CPU);

                    auto state_max = torch::max(q2, 1);
                    auto state_max_val = std::get<0>(state_max);
                    auto Y = reward_batch_t + GAMMA * ((1 - done_batch_t) * state_max_val);
                    auto X = q1.gather(1, action_batch_t.unsqueeze(1)).squeeze();
                    auto loss = loss_fn(X, Y.detach());

                    optimizer_ptr->zero_grad();
                    loss.backward();
                    optimizer_ptr->step();

                    BITRL_LOG_INFO << "Loss at epoch: " << loss.item<real_t>();
                    epoch_loss.push_back(loss.item<real_t>());
                }

                step_counter += 1;

                // update done
                done = time_step.done();

                if (done || step_counter > TOTAL_ITRS_PER_EPOCH)
                { //#Q
                    // done = true;
                    BITRL_LOG_INFO << "Reward: " << reward;
                    BITRL_LOG_INFO << "Finishing epoch at step: " << step_counter;
                    step_counter = 0;
                }
            }

            BITRL_LOG_INFO << "Epoch finished...";

            // get the epsilon
            /*auto eps = policy.eps_value();
            if(eps > 0.1){
                    eps -= 1/static_cast<real_t>(TOTAL_EPOCHS);
                    policy.set_eps_value(eps);
            }*/

            losses.push_back(bitrl::maths::mean(epoch_loss.begin(), epoch_loss.end()));
        }

        // play the greedy policy, the action of the largest Q value, for 100 episodes on random
        // boards,
        // at most 20 steps each, and write every board to episode.csv for
        // examples/tools/grid_world_gif.py: a row per step, the episode, the
        // action that led to the board (-1 for the first), the reward and the 64
        // values of the board
        {
            qnet->eval();
            torch::NoGradGuard no_grad;
            std::ofstream episode_file("episode.csv");
            episode_file << "# episode,action,reward,board\n";
            uint_t won = 0;
            for (uint_t episode = 0; episode < 100; ++episode)
            {
                auto time_step = env.reset();
                int action = -1;
                for (uint_t step = 0; step <= 20; ++step)
                {
                    const auto obs = flattened_observation(time_step.observation());
                    episode_file << episode << "," << action << "," << time_step.reward();
                    for (auto v : obs)
                    {
                        episode_file << "," << v;
                    }
                    episode_file << "\n";
                    if (time_step.done() || step == 20)
                    {
                        won += time_step.reward() > 0.0 ? 1 : 0;
                        break;
                    }
                    auto state =
                        bitrl::utils::pytorch::TorchAdaptor::to_torch(obs, bitrl::DeviceType::CPU);
                    action = static_cast<int>(torch::argmax(qnet(state)).item<int64_t>());
                    time_step = env.step(static_cast<uint_t>(action));
                }
            }
            std::cout << "The greedy policy reached the goal in " << won << " of " << 100
                      << " episodes" << std::endl;
        }

        // save the rewards per episode for visualization
        // purposes
        auto filename = std::string("experiments/") + EXPERIMENT_ID;
        filename += "/dqn_grid_world_policy_loss.csv";
        CSVWriter csv_writer(filename, CSVWriter::default_delimiter());
        csv_writer.open();
        csv_writer.write_column_vector(losses);
        csv_writer.close();
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << std::endl;
    }
    catch (...)
    {
        std::cout << "Unknown exception occured" << std::endl;
    }
    return 0;
}
#else
#include <iostream>
int main()
{

    std::cout << "This example requires PyTorch" << std::endl;
    std::cout << "Reconfigure bitrl with USE_PYTORCH flag turned ON." << std::endl;
    return 0;
}
#endif
