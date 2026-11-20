#include "bitrl/base/bitrl_types.h"
#include "bitrl/envs/time_step.h"
#include "bitrl/rl/algorithms/td/q_learning.h"
#include "bitrl/rl/algorithms/td/sarsa.h"
#include "bitrl/rl/policies/epsilon_greedy_policy.h"
#include "bitrl/rl/policies/max_tabular_policy.h"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace
{
using bitrl::real_t;
using bitrl::uint_t;
using bitrl::rl::policies::EpsilonGreedyPolicy;

///
/// The cliff walk of Sutton and Barto, example 6.6, on a 3 x 5 grid:
///
///     . . . . .
///     . . . . .
///     S C C C G
///
/// The agent starts at S and the episode ends at G. Every step costs 1;
/// stepping onto the cliff, C, costs 100 and ends the episode. Actions are
/// 0 up, 1 down, 2 left, 3 right; moving off the grid leaves the agent where
/// it is. States are row * 5 + column.
///
class CliffWalk
{
  public:
    static constexpr uint_t N_ROWS = 3;
    static constexpr uint_t N_COLS = 5;
    static constexpr uint_t START = (N_ROWS - 1) * N_COLS;
    static constexpr uint_t GOAL = N_ROWS * N_COLS - 1;

    typedef uint_t state_type;
    typedef uint_t action_type;
    typedef bitrl::TimeStep<uint_t> time_step_type;

    uint_t n_states() const noexcept { return N_ROWS * N_COLS; }
    uint_t n_actions() const noexcept { return 4; }

    static bool is_cliff(uint_t s)
    {
        return s / N_COLS == N_ROWS - 1 && s % N_COLS > 0 && s % N_COLS < N_COLS - 1;
    }

    static uint_t move(uint_t s, uint_t a)
    {
        uint_t row = s / N_COLS;
        uint_t col = s % N_COLS;
        if (a == 0 && row > 0)
            row -= 1;
        else if (a == 1 && row < N_ROWS - 1)
            row += 1;
        else if (a == 2 && col > 0)
            col -= 1;
        else if (a == 3 && col < N_COLS - 1)
            col += 1;
        return row * N_COLS + col;
    }

    time_step_type reset()
    {
        state_ = START;
        return time_step_type(bitrl::TimeStepTp::FIRST, 0.0, state_);
    }

    time_step_type step(action_type a)
    {
        state_ = move(state_, a);
        if (is_cliff(state_))
        {
            return time_step_type(bitrl::TimeStepTp::LAST, -100.0, state_);
        }
        if (state_ == GOAL)
        {
            return time_step_type(bitrl::TimeStepTp::LAST, -1.0, state_);
        }
        return time_step_type(bitrl::TimeStepTp::MID, -1.0, state_);
    }

  private:
    uint_t state_{START};
};

template <typename Solver> void train(Solver &solver, CliffWalk &env, uint_t n_episodes)
{
    solver.actions_before_training_begins(env);
    for (uint_t episode = 0; episode < n_episodes; ++episode)
    {
        solver.actions_before_episode_begins(env, episode);
        const auto info = solver.on_training_episode(env, episode);
        solver.actions_after_episode_ends(env, episode, info);
    }
}

// The states the greedy policy goes through from the start, up to max_steps
std::vector<uint_t> greedy_path(const bitrl::rl::policies::MaxTabularPolicy &policy,
                                uint_t max_steps = 30)
{
    std::vector<uint_t> path{CliffWalk::START};
    uint_t s = CliffWalk::START;
    for (uint_t i = 0; i < max_steps && s != CliffWalk::GOAL && !CliffWalk::is_cliff(s); ++i)
    {
        s = CliffWalk::move(s, policy.on_state(s));
        path.push_back(s);
    }
    return path;
}

std::string to_string(const std::vector<uint_t> &path)
{
    std::string out;
    for (auto s : path)
    {
        out += std::to_string(s) + " ";
    }
    return out;
}
} // namespace

namespace
{
bool reaches_the_goal(const std::vector<uint_t> &path) { return path.back() == CliffWalk::GOAL; }

} // namespace

// SARSA learns the value of the policy it follows, which explores, so it
// keeps away from the cliff: a random step from the edge would fall off
TEST(TestTDCliff, SarsaTakesTheSafePath)
{
    for (uint_t seed = 1; seed <= 5; ++seed)
    {
        CliffWalk env;
        bitrl::rl::algos::td::SarsaConfig config{500, 1e-8, 1.0, 0.5, 100};
        bitrl::rl::algos::td::SarsaSolver<CliffWalk, EpsilonGreedyPolicy> solver(
            config, EpsilonGreedyPolicy(0.1, seed));
        train(solver, env, 500);

        const auto path = greedy_path(solver.build_policy());
        ASSERT_TRUE(reaches_the_goal(path)) << "seed " << seed << ": " << to_string(path);
        bool by_the_top_row = false;
        for (auto s : path)
        {
            EXPECT_FALSE(CliffWalk::is_cliff(s)) << "seed " << seed << ": " << to_string(path);
            by_the_top_row = by_the_top_row || s / CliffWalk::N_COLS == 0;
        }
        // the shortest path, along the edge of the cliff, never goes there
        EXPECT_TRUE(by_the_top_row) << "seed " << seed << ": " << to_string(path);
    }
}

// Q-learning learns the value of the greedy policy, whatever it explores
// with, so it takes the shortest path, along the edge of the cliff
TEST(TestTDCliff, QLearningTakesTheShortestPath)
{
    for (uint_t seed = 1; seed <= 5; ++seed)
    {
        CliffWalk env;
        bitrl::rl::algos::td::QLearningConfig config;
        config.n_episodes = 500;
        config.max_num_iterations_per_episode = 100;
        config.tolerance = 1e-8;
        config.gamma = 1.0;
        config.eta = 0.5;
        bitrl::rl::algos::td::QLearningSolver<CliffWalk, EpsilonGreedyPolicy> solver(
            config, EpsilonGreedyPolicy(0.1, seed));
        train(solver, env, 500);

        const auto path = greedy_path(solver.build_policy());
        ASSERT_TRUE(reaches_the_goal(path)) << "seed " << seed << ": " << to_string(path);
        // up, four steps right along the edge, down
        EXPECT_EQ(path.size(), 7u) << "seed " << seed << ": " << to_string(path);
    }
}
