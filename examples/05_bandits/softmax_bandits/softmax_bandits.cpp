/**
 * Solve the multi-arm bandit problem using
 * soft-max policy.  When using a soft-max policy  policy
 * we get a distribution of probabilities over the actions. We select the action with the
 * highest probability.
 * For this example we will solve a 10-armed bandit problem, so N=10.
 *
 * This example is taken from the book: Reinforcement Learning in Action by Manning Publications.
 *
 * */

#include "bitrl/base/bitrl_types.h"
#include "bitrl/maths/vector_math.h"
#include "bitrl/rl/policies/softmax_policy.h"
#include "bitsim/utils/io/csv_file_writer.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>
#include <random>
#include <tuple>
#include <utility>

namespace exe
{

using bitrl::DynVec;
using bitrl::real_t;
using bitrl::uint_t;

// number of arms
const uint_t N = 10;

// how many experiments to run
const auto N_EXPERIMENTS = 500;

// the largest reward an arm pays: 1, plus 1 for each of 10 trials that succeed
const real_t MAX_REWARD = 11.0;

// temperature parameter for soft-max
const auto TAU = 0.7;

// seed for random number generator
const uint SEED = 42;

real_t get_reward(real_t prob, uint_t n = 10)
{

    auto reward = 1;

    for (uint_t i = 0; i < n; ++i)
    {
        auto prob_value = static_cast<real_t>(rand()) / static_cast<real_t>(RAND_MAX);
        if (prob_value < prob)
        {
            reward += 1;
        }
    }

    return reward;
}

void update_record(std::vector<std::vector<real_t>> &records, uint_t action, real_t r)
{

    auto new_r = (records[action][0] * records[action][1] + r) / (records[action][0] + 1);
    records[action][0] += 1;
    records[action][1] = new_r;
}

uint_t get_best_arm(const std::vector<std::vector<real_t>> &records)
{

    std::vector<real_t> values(records.size(), 0.0);
    for (uint_t i = 0; i < records.size(); ++i)
    {
        values[i] = records[i][1];
    }

    auto iterator_result = std::max_element(values.begin(), values.end());
    return std::distance(values.begin(), iterator_result);
}

std::vector<real_t> get_probs(uint_t n)
{

    std::vector<real_t> probs(n);

    for (uint_t i = 0; i < n; ++i)
    {
        probs[i] = static_cast<real_t>(rand()) / static_cast<real_t>(RAND_MAX);
    }

    return probs;
}

DynVec<real_t> extract_part(const std::vector<std::vector<real_t>> &values)
{

    auto vec = DynVec<real_t>(values.size());

    auto counter = 0;
    for (auto item : values)
    {
        vec[counter++] = item[1];
    }

    // std::for_each(values.begin(),
    //              values.end(),
    //              [&vec, &counter](const auto& item){vec[counter] = item[1]; counter++;});

    return vec; // DynVec<real_t>(values.size(), result.data());
}

} // namespace exe

int main()
{

    using namespace exe;

    auto probs = get_probs(N);

    std::vector<std::vector<real_t>> records(N);

    for (uint_t i = 0; i < N; ++i)
    {
        // the number of pulls of the arm, and its mean reward so far. The mean
        // starts at the largest reward an arm can pay, 1 + 10: with a mean of
        // 0, the first arm that paid would outweigh the untried ones so much
        // that the policy would pull it for ever. Each untried arm looks as
        // good as any can be, until it is tried
        records[i] = {0.0, MAX_REWARD};
    }

    // the rewards we accumulate
    std::vector<real_t> rewards;
    std::vector<uint_t> arms;
    std::vector<real_t> pulled;
    rewards.reserve(N_EXPERIMENTS);

    // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(SEED);

    for (uint_t i = 0; i < N_EXPERIMENTS; ++i)
    {

        std::cout << "Running experiment: " << i + 1 << std::endl;

        auto record = extract_part(records);

        // soft-max the vector
        auto p = bitrl::maths::softmax_vec(record.begin(), record.end(), TAU);

        std::discrete_distribution<> distribution(p.begin(), p.end());
        auto choice = distribution(gen);

        auto r = get_reward(probs[choice]);
        update_record(records, choice, r);
        arms.push_back(choice);
        pulled.push_back(r);

        if (!rewards.empty())
        {
            auto mean_reward = ((i + 1) * rewards.back() + r) / (i + 2);
            rewards.push_back(mean_reward);
        }
        else
        {
            rewards.push_back(r);
        }

        std::cout << "\tReward obtained: " << rewards[i] << std::endl;
    }

    // the arm and the reward of every pull, for examples/tools/bandit_gif.py,
    // and the success probability of every arm, for drawing them
    {
        std::ofstream file("pulls.csv");
        file << "# p";
        for (auto p : probs)
        {
            file << "," << p;
        }
        file << "\n# method,pull,arm,reward\n";
        for (uint_t i = 0; i < arms.size(); ++i)
        {
            file << "soft-max," << i << "," << arms[i] << "," << pulled[i] << "\n";
        }
    }

    auto csv_writer = bitsim::utils::io::CSVWriter("rewards.csv", ',');

    csv_writer.open();
    csv_writer.write_column_vector(rewards);
    csv_writer.close();

    return 0;
}
