#include "bitrl/envs/multi_agent/multi_agent_grid_world.h"
#include "bitrl/envs/multi_agent/multi_robot_navigation.h"
#include "bitrl/rl/multi_agent/independent_q_learning.h"
#include "bitrl/rl/multi_agent/multi_agent_trainer.h"

#include <gtest/gtest.h>

#include <any>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
using bitrl::real_t;
using bitrl::uint_t;
using namespace bitrl::envs::multi_agent;
using namespace bitrl::rl::multi_agent;

constexpr uint_t STAY = static_cast<uint_t>(GridMove::STAY);
constexpr uint_t UP = static_cast<uint_t>(GridMove::UP);
constexpr uint_t RIGHT = static_cast<uint_t>(GridMove::RIGHT);
constexpr uint_t DOWN = static_cast<uint_t>(GridMove::DOWN);
constexpr uint_t LEFT = static_cast<uint_t>(GridMove::LEFT);

MultiAgentGridWorld make_grid(const std::vector<std::string> &layout,
                              std::unordered_map<std::string, std::any> options = {})
{
    options["layout"] = layout;
    MultiAgentGridWorld env;
    env.make("v1", options, {});
    return env;
}

TEST(TestMultiAgentTimeStep, ChecksTheSizesAndSumsTheRewards)
{
    MultiAgentTimeStep<uint_t> step(bitrl::TimeStepTp::MID, {1, 2}, {0.5, -1.5}, {false, true});
    EXPECT_EQ(step.n_agents(), 2u);
    EXPECT_DOUBLE_EQ(step.total_reward(), -1.0);
    EXPECT_TRUE(step.done(1));
    EXPECT_FALSE(step.last());
    EXPECT_THROW(MultiAgentTimeStep<uint_t>(bitrl::TimeStepTp::MID, {1, 2}, {0.5}, {false, true}),
                 std::invalid_argument);
}

TEST(TestMultiAgentGridWorld, ReadsTheDefaultLayout)
{
    MultiAgentGridWorld env;
    env.make("v1", {}, {});
    EXPECT_EQ(env.n_agents(), 2u);
    EXPECT_EQ(env.n_rows(), 5u);
    EXPECT_EQ(env.n_cols(), 9u);
    EXPECT_EQ(env.n_states(), env.n_cells());
    EXPECT_EQ(env.agent_name(1), "B");

    const auto step = env.reset();
    EXPECT_TRUE(step.first());
    EXPECT_EQ(env.positions()[0], (MultiAgentGridWorld::cell_type{1, 1}));
    EXPECT_EQ(env.goals()[0], (MultiAgentGridWorld::cell_type{3, 7}));
    EXPECT_EQ(step.observation(0), env.cell_index({1, 1}));
}

TEST(TestMultiAgentGridWorld, JointObservationsEncodeEveryAgent)
{
    auto env = make_grid(MultiAgentGridWorld::default_layout(),
                         {{"observation", GridObservation::JOINT_POSITIONS}});
    EXPECT_EQ(env.n_states(), env.n_cells() * env.n_cells());
    const auto step = env.reset();
    const uint_t a = env.cell_index(env.positions()[0]);
    const uint_t b = env.cell_index(env.positions()[1]);
    EXPECT_EQ(step.observation(0), a + env.n_cells() * b);
    EXPECT_EQ(step.observation(1), b + env.n_cells() * a);
}

TEST(TestMultiAgentGridWorld, WallsStopAMove)
{
    auto env = make_grid({"####", "#Aa#", "####"});
    env.reset();
    const auto step = env.step({UP});
    EXPECT_EQ(env.positions()[0], (MultiAgentGridWorld::cell_type{1, 1}));
    EXPECT_DOUBLE_EQ(step.reward(0), -0.05);
    EXPECT_EQ(step.get_extra<uint_t>("collisions"), 0u);
}

TEST(TestMultiAgentGridWorld, ReachingTheGoalEndsTheEpisode)
{
    auto env = make_grid({"####", "#Aa#", "####"});
    env.reset();
    const auto step = env.step({RIGHT});
    EXPECT_TRUE(step.done(0));
    EXPECT_TRUE(step.last());
    EXPECT_FALSE(step.truncated());
    EXPECT_DOUBLE_EQ(step.reward(0), -0.05 + 1.0);
}

TEST(TestMultiAgentGridWorld, TwoAgentsCannotEnterOneCell)
{
    // A and B both step into the middle cell
    auto env = make_grid({"#####", "#A.B#", "#a.b#", "#####"});
    env.reset();
    const auto step = env.step({RIGHT, LEFT});
    EXPECT_EQ(env.positions()[0], (MultiAgentGridWorld::cell_type{1, 1}));
    EXPECT_EQ(env.positions()[1], (MultiAgentGridWorld::cell_type{1, 3}));
    EXPECT_EQ(step.get_extra<uint_t>("collisions"), 2u);
    EXPECT_DOUBLE_EQ(step.reward(0), -0.05 - 0.5);
    EXPECT_EQ(env.n_collisions(), 2u);
}

TEST(TestMultiAgentGridWorld, TwoAgentsCannotSwapCells)
{
    auto env = make_grid({"####", "#AB#", "#ba#", "####"});
    env.reset();
    const auto step = env.step({RIGHT, LEFT});
    EXPECT_EQ(env.positions()[0], (MultiAgentGridWorld::cell_type{1, 1}));
    EXPECT_EQ(env.positions()[1], (MultiAgentGridWorld::cell_type{1, 2}));
    EXPECT_EQ(step.get_extra<uint_t>("collisions"), 2u);
}

TEST(TestMultiAgentGridWorld, AnAgentMayFollowAnother)
{
    auto env = make_grid({"######", "#AB..#", "#..ba#", "######"});
    env.reset();
    const auto step = env.step({RIGHT, RIGHT});
    EXPECT_EQ(env.positions()[0], (MultiAgentGridWorld::cell_type{1, 2}));
    EXPECT_EQ(env.positions()[1], (MultiAgentGridWorld::cell_type{1, 3}));
    EXPECT_EQ(step.get_extra<uint_t>("collisions"), 0u);
}

TEST(TestMultiAgentGridWorld, ABlockedLeaderBlocksItsFollower)
{
    // B cannot move into the wall, so A cannot take B's cell either
    auto env = make_grid({"####", "#AB#", "#ab#", "####"});
    env.reset();
    const auto step = env.step({RIGHT, RIGHT});
    EXPECT_EQ(env.positions()[0], (MultiAgentGridWorld::cell_type{1, 1}));
    EXPECT_EQ(env.positions()[1], (MultiAgentGridWorld::cell_type{1, 2}));
    // only A tried to move into a cell that stayed taken
    EXPECT_EQ(step.get_extra<uint_t>("collisions"), 1u);
    EXPECT_DOUBLE_EQ(step.reward(1), -0.05);
}

TEST(TestMultiAgentGridWorld, SharedRewardGivesEveryoneTheTeamReward)
{
    auto env = make_grid({"#####", "#Aa.#", "#B.b#", "#####"}, {{"shared_reward", true}});
    env.reset();
    const auto step = env.step({RIGHT, STAY});
    EXPECT_DOUBLE_EQ(step.reward(0), (-0.05 + 1.0) + (-0.05));
    EXPECT_DOUBLE_EQ(step.reward(1), step.reward(0));
}

TEST(TestMultiAgentGridWorld, TheStepLimitTruncates)
{
    auto env = make_grid({"#####", "#A.a#", "#####"}, {{"max_steps", uint_t(3)}});
    env.reset();
    env.step({STAY});
    env.step({STAY});
    const auto step = env.step({STAY});
    EXPECT_TRUE(step.last());
    EXPECT_TRUE(step.truncated());
    EXPECT_FALSE(step.done(0));
}

TEST(TestMultiAgentGridWorld, RejectsBadInput)
{
    MultiAgentGridWorld env;
    EXPECT_THROW(env.step({STAY}), std::logic_error);
    EXPECT_THROW(make_grid({"###", "#A#", "###"}), std::invalid_argument); // no goal
    auto ok = make_grid({"####", "#Aa#", "####"});
    ok.reset();
    EXPECT_THROW(ok.step({STAY, STAY}), std::invalid_argument);
    EXPECT_THROW(ok.step({7}), std::invalid_argument);
}

TEST(TestMultiAgentGridWorld, RendersAgentsAndGoals)
{
    auto env = make_grid({"#####", "#A.a#", "#####"});
    env.reset();
    EXPECT_EQ(env.render(), "#####\n#A.a#\n#####\n");
    env.step({RIGHT});
    EXPECT_EQ(env.render(), "#####\n#.Aa#\n#####\n");
}

TEST(TestMultiRobotNavigation, StartsOnACircleFacingTheCentre)
{
    MultiRobotNavigation env;
    env.make("v1", {{"n_robots", uint_t(4)}, {"circle_radius", 2.0}}, {});
    const auto step = env.reset();
    EXPECT_EQ(step.n_agents(), 4u);
    EXPECT_EQ(step.observation(0).size(), env.observation_size());
    EXPECT_EQ(env.observation_size(), 5u + 4u * 3u);
    EXPECT_NEAR(env.poses()[0].x, 2.0, 1e-12);
    EXPECT_NEAR(std::abs(env.poses()[0].theta), M_PI, 1e-12);
    EXPECT_NEAR(env.goals()[0][0], -2.0, 1e-12);
    EXPECT_NEAR(env.distance_to_goal(1), 4.0, 1e-12);
}

TEST(TestMultiRobotNavigation, DrivingStraightGainsTheDistance)
{
    MultiRobotNavigation env;
    env.make("v1", {{"n_robots", uint_t(1)}, {"dt", 0.1}}, {});
    env.reset();
    const auto step = env.step({{0.5, 0.0}});
    EXPECT_NEAR(env.distance_to_goal(0), 4.0 - 0.05, 1e-12);
    EXPECT_NEAR(step.reward(0), 0.05 - 0.01, 1e-12);
    EXPECT_NEAR(step.observation(0)[0], 2.0 - 0.05, 1e-12);
}

TEST(TestMultiRobotNavigation, ClipsTheVelocities)
{
    MultiRobotNavigation env;
    env.make("v1", {{"n_robots", uint_t(1)}, {"max_v", 0.5}}, {});
    env.reset();
    env.step({{10.0, 0.0}});
    EXPECT_NEAR(env.distance_to_goal(0), 4.0 - 0.05, 1e-12);
}

TEST(TestMultiRobotNavigation, RobotsDoNotOverlap)
{
    // two robots head straight for each other, and stop when they touch
    MultiRobotNavigation env;
    env.make("v1", {{"n_robots", uint_t(2)}, {"circle_radius", 0.5}, {"robot_radius", 0.15}}, {});
    env.reset();
    uint_t collisions = 0;
    for (int k = 0; k < 20; ++k)
    {
        const auto step = env.step({{0.5, 0.0}, {0.5, 0.0}});
        collisions += step.get_extra<uint_t>("collisions");
    }
    EXPECT_GT(collisions, 0u);
    EXPECT_GE(env.min_separation(), 2.0 * 0.15 - 1e-12);
    EXPECT_EQ(env.n_collisions(), collisions);
}

TEST(TestMultiRobotNavigation, ARobotAtItsGoalIsDone)
{
    MultiRobotNavigation env;
    env.make("v1", {{"n_robots", uint_t(1)}, {"circle_radius", 0.1}, {"goal_tolerance", 0.1}}, {});
    env.reset();
    auto step = env.step({{0.5, 0.0}});
    for (int k = 0; k < 10 && !step.last(); ++k)
    {
        step = env.step({{0.5, 0.0}});
    }
    EXPECT_TRUE(step.done(0));
    EXPECT_TRUE(step.last());
}

TEST(TestMultiRobotNavigation, StartNoiseDependsOnTheSeed)
{
    MultiRobotNavigation env;
    env.make("v1", {{"start_noise", 0.05}}, {{"seed", uint_t(7)}});
    env.reset(3);
    const real_t x3 = env.poses()[0].x;
    env.reset(4);
    const real_t x4 = env.poses()[0].x;
    env.reset(3);
    EXPECT_DOUBLE_EQ(env.poses()[0].x, x3);
    EXPECT_NE(x3, x4);
}

TEST(TestMultiRobotNavigation, RejectsTooManyRobotsForTheCircle)
{
    MultiRobotNavigation env;
    EXPECT_THROW(env.make("v1", {{"n_robots", uint_t(40)}, {"circle_radius", 0.5}}, {}),
                 std::invalid_argument);
}

TEST(TestIndependentQLearning, LearnsASingleCorridor)
{
    auto env = make_grid({"######", "#A..a#", "######"});
    IndependentQLearningConfig config;
    config.eps_decay = 0.05;
    IndependentQLearning learner(env.n_agents(), env.n_states(), env.n_actions(), config);
    MultiAgentTrainer<MultiAgentGridWorld, IndependentQLearning> trainer({200}, learner);
    const auto training = trainer.train(env);
    EXPECT_EQ(training.episodes.size(), 200u);
    EXPECT_LT(learner.epsilon(), 0.1);

    auto greedy = learner.greedy();
    const auto result = run_episode(env, greedy);
    EXPECT_TRUE(result.all_done);
    EXPECT_EQ(result.n_steps, 3u);
}

TEST(TestIndependentQLearning, TheUpdateFollowsQLearning)
{
    IndependentQLearningConfig config;
    config.gamma = 0.5;
    config.eta = 0.5;
    IndependentQLearning learner(1, 3, 2, config);
    using step_type = MultiAgentTimeStep<uint_t>;
    const step_type s0(bitrl::TimeStepTp::FIRST, {0}, {0.0}, {false});
    const step_type s1(bitrl::TimeStepTp::MID, {1}, {1.0}, {false});
    learner.update(s0, {1}, s1);
    // Q(0, 1) = 0 + 0.5 (1 + 0.5 * 0 - 0)
    EXPECT_DOUBLE_EQ(learner.q_table(0)(0, 1), 0.5);

    // a terminal transition does not bootstrap
    const step_type s2(bitrl::TimeStepTp::LAST, {2}, {2.0}, {true});
    learner.update(s1, {0}, s2);
    EXPECT_DOUBLE_EQ(learner.q_table(0)(1, 0), 1.0);

    // a done agent learns nothing more
    learner.update(s2, {0}, s2);
    EXPECT_DOUBLE_EQ(learner.q_table(0)(2, 0), 0.0);
}

TEST(TestIndependentQLearning, IsRepeatableForASeed)
{
    auto train = [](uint_t seed)
    {
        MultiAgentGridWorld env;
        env.make("v1", {}, {});
        IndependentQLearningConfig config;
        config.seed = seed;
        IndependentQLearning learner(env.n_agents(), env.n_states(), env.n_actions(), config);
        MultiAgentTrainer<MultiAgentGridWorld, IndependentQLearning> trainer({50}, learner);
        trainer.train(env);
        return learner.q_table(0);
    };
    EXPECT_TRUE(train(3).isApprox(train(3)));
    EXPECT_FALSE(train(3).isApprox(train(4)));
}

} // namespace
