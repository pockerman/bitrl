#include "bitrl/base/bitrl_types.h"
#include "bitrl/envs/time_step.h"
#include "bitrl/rl/agents/dummy_agent.h"
#include "bitrl/rl/algorithms/td/double_q_learning.h"
#include "bitrl/rl/policies/epsilon_double_qtable_greedy_policy.h"
#include "bitsim/utils/iteration_counter.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>

namespace
{
using bitrl::DynMat;
using bitrl::real_t;
using bitrl::uint_t;

///
/// A chain of N states. The agent starts in state 0; action 0 moves left and
/// action 1 right. Reaching the last state ends the episode with a reward of 1;
/// every other step gives 0.
///
template <uint_t N> class Chain
{
  public:
    typedef uint_t state_type;
    typedef uint_t action_type;
    typedef bitrl::TimeStep<uint_t> time_step_type;

    uint_t n_states() const noexcept { return N; }
    uint_t n_actions() const noexcept { return 2; }

    time_step_type reset()
    {
        state_ = 0;
        return time_step_type(bitrl::TimeStepTp::FIRST, 0.0, state_);
    }

    time_step_type step(action_type action)
    {
        if (action == 1)
        {
            state_ += 1;
        }
        else if (state_ > 0)
        {
            state_ -= 1;
        }

        if (state_ == N - 1)
        {
            return time_step_type(bitrl::TimeStepTp::LAST, 1.0, state_);
        }
        return time_step_type(bitrl::TimeStepTp::MID, 0.0, state_);
    }

  private:
    uint_t state_{0};
};

using env_type = Chain<5>;
using policy_type = bitrl::rl::policies::EpsilonDoubleQTableGreedyPolicy<DynMat<real_t>>;
using solver_type = bitrl::rl::algos::td::DoubleQLearning<env_type, policy_type>;

bitrl::rl::algos::td::DoubleQLearningConfig make_config()
{
    bitrl::rl::algos::td::DoubleQLearningConfig config;
    config.gamma = 0.9;
    config.eta = 0.5;
    config.max_num_iterations_per_episode = 100;
    return config;
}

policy_type make_policy()
{
    return policy_type(0.3, 2, bitrl::rl::EpsilonDecayOptionType::NONE, 0.01, 1.0, 0.2, 42);
}

// the solvers cannot be copied, so the test makes one and this trains it
void train(solver_type &solver, env_type &env, uint_t n_episodes)
{
    solver.actions_before_training_begins(env);
    for (uint_t episode = 0; episode < n_episodes; ++episode)
    {
        solver.actions_before_episode_begins(env, episode);
        const auto info = solver.on_training_episode(env, episode);
        solver.actions_after_episode_ends(env, episode, info);
    }
    solver.actions_after_training_ends(env);
}
} // namespace

TEST(TestDoubleQLearning, LearnsToMoveRight)
{
    env_type env;
    solver_type solver(make_config(), make_policy());
    train(solver, env, 300);

    const DynMat<real_t> sum = solver.q_table_1() + solver.q_table_2();
    for (uint_t s = 0; s < env.n_states() - 1; ++s)
    {
        EXPECT_GT(sum(s, 1), sum(s, 0)) << "in state " << s;
    }
}

TEST(TestDoubleQLearning, UpdatesBothTables)
{
    // the coin that picks the table to update must land on both sides
    env_type env;
    solver_type solver(make_config(), make_policy());
    train(solver, env, 50);
    EXPECT_GT(solver.q_table_1().cwiseAbs().sum(), 0.0);
    EXPECT_GT(solver.q_table_2().cwiseAbs().sum(), 0.0);
}

TEST(TestDoubleQLearning, TheValueOfTheStepIntoTheGoalApproachesTheReward)
{
    // from the state before the goal, moving right ends the episode with a
    // reward of 1, so its value tends to 1 with nothing added after it
    env_type env;
    solver_type solver(make_config(), make_policy());
    train(solver, env, 300);
    EXPECT_NEAR(solver.q_table_1()(3, 1), 1.0, 1.0e-3);
    EXPECT_NEAR(solver.q_table_2()(3, 1), 1.0, 1.0e-3);
}

TEST(TestDoubleQLearning, SavesBothTables)
{
    env_type env;
    solver_type solver(make_config(), make_policy());
    train(solver, env, 10);
    const auto path = std::filesystem::temp_directory_path() / "test_double_q_learning.csv";
    solver.save(path.string());

    // the CSV writer starts with comment lines, the column names among them
    std::ifstream file(path);
    std::string line;
    bool found_names = false;
    uint_t n_rows = 0;
    while (std::getline(file, line))
    {
        if (!line.empty() && line[0] == '#')
        {
            found_names = found_names ||
                          line.find("state_index,table,action_0,action_1") != std::string::npos;
            continue;
        }
        // the state, the table and the value of each of the two actions
        EXPECT_EQ(std::count(line.begin(), line.end(), ','), 3) << line;
        n_rows += 1;
    }
    EXPECT_TRUE(found_names);
    // a row per state and table
    EXPECT_EQ(n_rows, 2 * env.n_states());
    std::filesystem::remove(path);
}

namespace
{
struct AlwaysRight
{
    uint_t on_state(uint_t /*state*/) const { return 1; }
};

// counts the steps it was given and how many reached the goal
class CountingChain : public env_type
{
  public:
    time_step_type step(action_type action)
    {
        auto time_step = env_type::step(action);
        n_steps += 1;
        n_goals += time_step.done() ? 1 : 0;
        return time_step;
    }
    uint_t n_steps{0};
    uint_t n_goals{0};
};
} // namespace

TEST(TestDummyAgent, PlaysThePolicyAndResetsAtTheEnd)
{
    CountingChain env;
    AlwaysRight policy;
    bitrl::rl::agents::DummyAgent<CountingChain, AlwaysRight> agent(policy);
    bitsim::utils::IterationCounter counter(12);
    agent.play(env, counter);

    // 4 steps right reach the goal of a chain of 5; 12 steps reach it 3 times
    EXPECT_EQ(env.n_steps, 12u);
    EXPECT_EQ(env.n_goals, 3u);
}
