#include "bitrl/base/bitrl_types.h"
#include "bitrl/rl/algorithms/mc/mc_tree_search_solver.h"

#include <gtest/gtest.h>

namespace
{
using bitrl::real_t;
using bitrl::uint_t;
using bitrl::rl::algos::mc::MCTSConfig;
using bitrl::rl::algos::mc::MCTSSolver;

///
/// A chain of states 0 to N - 1. Action 0 moves left, action 1 right. The last
/// state ends the episode with a reward of 1; falling off the left end, from
/// state 0, ends it with a reward of -1. Every other step costs 0.01.
///
class Chain
{
  public:
    typedef uint_t action_type;
    typedef uint_t state_type;

    struct TimeStep
    {
        real_t reward_;
        bool done_;
        real_t reward() const noexcept { return reward_; }
        bool done() const noexcept { return done_; }
    };

    Chain(uint_t n, uint_t start) : n_(n), state_(start) {}

    uint_t n_actions() const noexcept { return 2; }
    uint_t state() const noexcept { return state_; }

    TimeStep step(action_type action)
    {
        if (action == 0)
        {
            if (state_ == 0)
            {
                return {-1.0, true};
            }
            state_ -= 1;
            return {-0.01, false};
        }
        state_ += 1;
        if (state_ == n_ - 1)
        {
            return {1.0, true};
        }
        return {-0.01, false};
    }

  private:
    uint_t n_;
    uint_t state_;
};

MCTSConfig config(uint_t n_simulations, uint_t max_depth)
{
    MCTSConfig c;
    c.n_simulations = n_simulations;
    c.max_depth = max_depth;
    c.temperature = 1.0;
    c.gamma = 0.95;
    c.seed = 3;
    return c;
}
} // namespace

TEST(TestMCTS, MovesTowardsTheGoal)
{
    MCTSSolver<Chain> solver(config(300, 20));
    for (uint_t start = 0; start < 6; ++start)
    {
        EXPECT_EQ(solver.plan(Chain(8, start)), 1u) << "from state " << start;
    }
}

TEST(TestMCTS, AvoidsTheCliff)
{
    // from state 0 moving left falls off at once; the goal is 3 steps away
    MCTSSolver<Chain> solver(config(200, 10));
    EXPECT_EQ(solver.plan(Chain(4, 0)), 1u);
}

TEST(TestMCTS, EverySimulationVisitsTheRoot)
{
    MCTSSolver<Chain> solver(config(150, 10));
    solver.plan(Chain(8, 2));
    ASSERT_TRUE(solver.root());
    EXPECT_EQ(solver.root()->total_visits(), 150u);
    EXPECT_EQ(solver.root()->n_children(), 2u);
    EXPECT_EQ(solver.root()->get_child(0)->total_visits() +
                  solver.root()->get_child(1)->total_visits(),
              150u);
}

TEST(TestMCTS, DoesNotChangeTheEnvironment)
{
    MCTSSolver<Chain> solver(config(50, 10));
    const Chain env(8, 3);
    solver.plan(env);
    EXPECT_EQ(env.state(), 3u);
}

TEST(TestMCTS, TheSameSeedGivesTheSamePlan)
{
    MCTSSolver<Chain> a(config(100, 10));
    MCTSSolver<Chain> b(config(100, 10));
    for (uint_t start = 0; start < 6; ++start)
    {
        EXPECT_EQ(a.plan(Chain(8, start)), b.plan(Chain(8, start)));
        EXPECT_EQ(a.root()->get_child(0)->total_visits(), b.root()->get_child(0)->total_visits());
    }
}
