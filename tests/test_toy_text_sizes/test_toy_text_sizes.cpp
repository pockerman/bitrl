// The numbers of states and actions of the toy text environments must be
// those of Gymnasium's, since the tabular solvers size their tables with them.
// They are known without the server, so this needs none.
#include "bitrl/envs/gymnasium/toy_text/cliff_world_env.h"
#include "bitrl/envs/gymnasium/toy_text/frozen_lake_env.h"
#include "bitrl/envs/gymnasium/toy_text/taxi_env.h"
#include "bitrl/network/rest_rl_env_client.h"

#include <gtest/gtest.h>

namespace
{
using namespace bitrl::envs::gymnasium;
bitrl::network::RESTRLEnvClient server("http://0.0.0.0:8001/api", false);
} // namespace

TEST(TestToyTextSizes, FrozenLake)
{
    // 4 actions: left, down, right, up
    EXPECT_EQ(FrozenLake<4>(server).n_states(), 16u);
    EXPECT_EQ(FrozenLake<4>(server).n_actions(), 4u);
    EXPECT_EQ(FrozenLake<8>(server).n_states(), 64u);
    EXPECT_EQ(FrozenLake<8>(server).n_actions(), 4u);
}

TEST(TestToyTextSizes, CliffWalking)
{
    // 4 x 12 cells; up, right, down, left
    EXPECT_EQ(CliffWorld(server).n_states(), 48u);
    EXPECT_EQ(CliffWorld(server).n_actions(), 4u);
}

TEST(TestToyTextSizes, Taxi)
{
    EXPECT_EQ(Taxi(server).n_states(), 500u);
    EXPECT_EQ(Taxi(server).n_actions(), 6u);
}
