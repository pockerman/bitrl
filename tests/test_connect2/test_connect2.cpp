#include "bitrl/bitrl_types.h"
#include "cuberl/envs/connect2/connect2_env.h"

#include <gtest/gtest.h>

#include <any>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
using bitrl::uint_t;
using cuberl::envs::connect2::Connect2;

Connect2 make_game()
{
    Connect2 env;
    const std::unordered_map<std::string, std::any> options;
    env.make("v1", options, options);
    env.reset();
    return env;
}
} // namespace

TEST(TestConnect2, TwoStonesApartDoNotWin)
{
    auto env = make_game();
    env.move(1, 0);
    const auto step = env.move(1, 2);
    EXPECT_FALSE(env.is_win(1));
    EXPECT_TRUE(step.mid());
    EXPECT_DOUBLE_EQ(step.reward(), 0.0);
}

TEST(TestConnect2, TwoStonesNextToEachOtherWin)
{
    auto env = make_game();
    env.move(1, 1);
    env.move(2, 3);
    const auto step = env.move(1, 2);
    EXPECT_TRUE(env.is_win(1));
    EXPECT_FALSE(env.is_win(2));
    EXPECT_TRUE(step.last());
    EXPECT_DOUBLE_EQ(step.reward(), 1.0);
}

TEST(TestConnect2, AFullBoardWithNoWinnerIsADraw)
{
    auto env = make_game();
    env.move(1, 0);
    env.move(2, 1);
    env.move(1, 2);
    const auto step = env.move(2, 3);
    EXPECT_FALSE(env.is_win(1));
    EXPECT_FALSE(env.is_win(2));
    EXPECT_TRUE(step.last());
    EXPECT_DOUBLE_EQ(step.reward(), 0.0);
}

TEST(TestConnect2, ValidMovesAndInvalidOnes)
{
    auto env = make_game();
    const auto step = env.move(1, 1);
    EXPECT_EQ(step.get_extra<std::vector<uint_t>>("valid_moves"), (std::vector<uint_t>{0, 2, 3}));
    EXPECT_THROW(env.move(2, 1), std::logic_error); // taken
    EXPECT_THROW(env.move(2, 4), std::logic_error); // no such cell
    EXPECT_THROW(env.move(3, 0), std::logic_error); // no such player
}
