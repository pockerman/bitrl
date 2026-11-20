#include "bitrl/base/bitrl_types.h"
#include "bitrl/base/torch_tensor_utils.h"
#include "bitrl/rl/algorithms/mc/mcts_node.h"
#include "bitrl/rl/algorithms/rl_algo_config.h"
#include "bitrl/rl/worlds/with_dynamics_mixin.h"
#include "bitrl/utils/bitrl_traits.h"
#include "bitrl/utils/lambda_utils.h"
#include "bitrl/utils/loss_type.h"

#include <gtest/gtest.h>
#include <torch/torch.h>

#include <algorithm>
#include <filesystem>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace
{
using bitrl::DynVec;
using bitrl::real_t;
using bitrl::uint_t;
} // namespace

TEST(TestBitrlTraits, GiveTheValueTypeOfAVector)
{
    using bitrl::utils::vector_value_type_trait;
    static_assert(std::is_same_v<vector_value_type_trait<std::vector<int>>::value_type, int>);
    static_assert(std::is_same_v<vector_value_type_trait<DynVec<real_t>>::value_type, real_t>);
    static_assert(
        std::is_same_v<vector_value_type_trait<std::pair<std::vector<float>, uint_t>>::value_type,
                       float>);
    SUCCEED();
}

TEST(TestLambdaUtils, PrintsTheValue)
{
    testing::internal::CaptureStdout();
    bitrl::utils::bitrl_print<int>(42);
    EXPECT_EQ(testing::internal::GetCapturedStdout(), "42\n");
}

TEST(TestLossType, ToStringAndCompare)
{
    using bitrl::utils::LossType;
    EXPECT_EQ(bitrl::utils::to_string(LossType::CROSS_ENTROPY), "CROSS_ENTROPY");
    EXPECT_EQ(bitrl::utils::to_string(LossType::MSE), "MSE");
    EXPECT_EQ(bitrl::utils::to_string(LossType::INVALID_TYPE), "INVALID_TYPE");
    static_assert(bitrl::utils::compare(LossType::MSE, LossType::MSE));
    static_assert(!bitrl::utils::compare(LossType::MSE, LossType::CROSS_ENTROPY));
}

TEST(TestRLAlgoConfig, KeepsItsValues)
{
    bitrl::rl::algos::RLAlgoConfig config{100, 10, 0, 5, 1.0e-4};
    EXPECT_EQ(config.n_episodes, 100u);
    EXPECT_EQ(config.n_itrs_per_episode, 10u);
    EXPECT_FALSE(config.render_environment);
}

namespace
{
// one state that loops back to itself with probability 1
struct OneStateWorld final : public bitrl::rl::envs::with_dynamics_mixin
{
    std::vector<std::tuple<real_t, uint_t, real_t, bool>>
    transition_dynamics(uint_t s, uint_t /*aidx*/) const override
    {
        return {{1.0, s, 0.5, false}};
    }
};
} // namespace

TEST(TestWithDynamicsMixin, GivesTheTransitions)
{
    static_assert(OneStateWorld::has_dynamics);
    const OneStateWorld world;
    const bitrl::rl::envs::with_dynamics_mixin &base = world;
    const auto transitions = base.transition_dynamics(0, 1);
    ASSERT_EQ(transitions.size(), 1u);
    EXPECT_DOUBLE_EQ(std::get<0>(transitions[0]), 1.0);
    EXPECT_EQ(std::get<1>(transitions[0]), 0u);
}

namespace
{
using node_type = bitrl::rl::algos::mc::MCTSNodeBase<uint_t, uint_t>;
} // namespace

TEST(TestMCTSNode, KeepsChildrenAndPicksTheBestUCB)
{
    auto root = std::make_shared<node_type>(nullptr, 0);
    root->add_child(root, 0);
    root->add_child(root, 1);
    root->add_child(root, 2);
    ASSERT_EQ(root->n_children(), 3u);
    EXPECT_TRUE(root->has_children());

    // a child that was never visited is explored first
    EXPECT_EQ(root->get_child(0)->ucb(1.0), std::numeric_limits<real_t>::max());

    for (uint_t i = 0; i < 10; ++i)
    {
        root->update_visits();
    }
    // child 0: 4 visits, score 1; child 1: 4 visits, score 3; child 2: 2 visits, score 1
    const std::vector<std::pair<uint_t, real_t>> stats = {{4, 1.0}, {4, 3.0}, {2, 1.0}};
    for (uint_t c = 0; c < 3; ++c)
    {
        for (uint_t v = 0; v < stats[c].first; ++v)
        {
            root->get_child(c)->update_visits();
        }
        root->get_child(c)->update_total_score(stats[c].second);
    }

    EXPECT_DOUBLE_EQ(root->get_child(1)->win_pct(), 0.75);
    EXPECT_NEAR(root->get_child(1)->ucb(1.0), 0.75 + std::sqrt(std::log(10.0) / 4.0), 1.0e-12);

    // without exploration the best win rate wins; with a lot of it the least
    // visited child does
    EXPECT_EQ(root->max_ucb_child(0.0)->get_action(), 1u);
    EXPECT_EQ(root->max_ucb_child(10.0)->get_action(), 2u);
}

TEST(TestMCTSNode, ShufflesItsChildren)
{
    auto root = std::make_shared<node_type>(nullptr, 0);
    for (uint_t a = 0; a < 6; ++a)
    {
        root->add_child(root, a);
    }
    std::mt19937 engine(1);
    root->shuffle_children(engine);

    std::vector<uint_t> actions;
    for (uint_t c = 0; c < root->n_children(); ++c)
    {
        actions.push_back(root->get_child(c)->get_action());
    }
    std::vector<uint_t> sorted = actions;
    std::sort(sorted.begin(), sorted.end());
    EXPECT_EQ(sorted, (std::vector<uint_t>{0, 1, 2, 3, 4, 5}));
}

TEST(TestTorchTensorUtils, CreateMask)
{
    const std::vector<int> values = {1, 0, 3, 0};
    const auto mask = bitrl::torch_utils::create_mask(values);
    EXPECT_EQ(mask.scalar_type(), torch::kBool);
    ASSERT_EQ(mask.numel(), 4);
    EXPECT_TRUE(mask[0].item<bool>());
    EXPECT_FALSE(mask[1].item<bool>());
    EXPECT_TRUE(mask[2].item<bool>());
    EXPECT_FALSE(mask[3].item<bool>());
}

namespace
{
struct NetImpl : torch::nn::Module
{
    NetImpl() : fc(register_module("fc", torch::nn::Linear(3, 2))) {}
    torch::nn::Linear fc;
};
TORCH_MODULE(Net);

bool same_parameters(const Net &a, const Net &b)
{
    const auto pa = a->named_parameters();
    const auto pb = b->named_parameters();
    for (const auto &item : pa)
    {
        if (!torch::equal(item.value(), pb[item.key()]))
        {
            return false;
        }
    }
    return true;
}
} // namespace

TEST(TestTorchTensorUtils, CopyParameters)
{
    torch::manual_seed(1);
    Net from;
    Net to;
    ASSERT_FALSE(same_parameters(from, to));
    bitrl::torch_utils::copy_parameters(from, to);
    EXPECT_TRUE(same_parameters(from, to));
}

TEST(TestTorchTensorUtils, CopyParametersFromAFile)
{
    torch::manual_seed(2);
    Net from;
    const auto path = std::filesystem::temp_directory_path() / "test_rl_utils_net.pt";
    torch::save(from, path.string());

    Net to;
    ASSERT_FALSE(same_parameters(from, to));
    bitrl::torch_utils::copy_parameters_to(to, path.string());
    EXPECT_TRUE(same_parameters(from, to));
    std::filesystem::remove(path);
}
