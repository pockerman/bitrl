#include "bitrl/envs/multi_agent/distributed_agent_env.h"
#include "bitrl/envs/multi_agent/multi_agent_grid_world.h"
#include "bitrl/envs/multi_agent/multi_robot_navigation.h"
#include "bitrl/rl/multi_agent/agent_trainer.h"
#include "bitrl/rl/multi_agent/independent_q_learning.h"
#include "bitrl/rl/multi_agent/multi_agent_trainer.h"
#include "bitrl/rl/multi_agent/q_learning_agent.h"
#include "bitsim/distributed/transport.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
using bitrl::real_t;
using bitrl::uint_t;
using namespace bitrl::envs::multi_agent;
using namespace bitrl::rl::multi_agent;
using bitsim::distributed::InProcessBus;
using bitsim::distributed::LossyTransport;
using bitsim::distributed::Transport;

MultiAgentGridWorld make_grid()
{
    MultiAgentGridWorld env;
    env.make("v1", {{"observation", GridObservation::JOINT_POSITIONS}, {"shared_reward", true}},
             {{"seed", uint_t(5)}});
    return env;
}

IndependentQLearningConfig q_config(uint_t n_episodes)
{
    IndependentQLearningConfig config;
    config.seed = 11;
    config.eta = 0.2;
    config.eps_decay = 5.0 / n_episodes;
    return config;
}

// Train every agent of the grid world in a thread of its own, over the given transports
std::vector<QLearningAgent> train_distributed(std::vector<std::unique_ptr<Transport>> &transports,
                                              uint_t n_episodes,
                                              std::vector<MultiAgentTrainingResult> &results)
{
    const uint_t n = make_grid().n_agents();
    std::vector<QLearningAgent> agents;
    for (uint_t a = 0; a < n; ++a)
    {
        const auto env = make_grid();
        agents.emplace_back(a, env.n_states(), env.n_actions(), q_config(n_episodes));
    }
    results.assign(n, {});
    std::vector<std::thread> threads;
    for (uint_t a = 0; a < n; ++a)
    {
        threads.emplace_back(
            [&, a]
            {
                auto env = make_grid();
                DistributedAgentEnv<MultiAgentGridWorld> mine(env, a, *transports[a]);
                AgentTrainer<DistributedAgentEnv<MultiAgentGridWorld>, QLearningAgent> trainer(
                    {n_episodes}, agents[a]);
                results[a] = trainer.train(mine);
                mine.leave();
            });
    }
    for (auto &t : threads)
    {
        t.join();
    }
    return agents;
}

TEST(TestAgentValues, RoundTrips)
{
    std::vector<real_t> v;
    AgentValues<uint_t>::append(3, v);
    AgentValues<GridMove>::append(GridMove::LEFT, v);
    AgentValues<std::array<real_t, 2>>::append({0.5, -1.0}, v);
    ASSERT_EQ(v.size(), 4u);
    EXPECT_EQ(AgentValues<uint_t>::read(v, 0, 1), 3u);
    EXPECT_EQ(AgentValues<GridMove>::read(v, 1, 1), GridMove::LEFT);
    EXPECT_EQ((AgentValues<std::array<real_t, 2>>::read(v, 2, 2)),
              (std::array<real_t, 2>{0.5, -1.0}));
    EXPECT_THROW((AgentValues<std::array<real_t, 2>>::read(v, 2, 1)), std::invalid_argument);
    EXPECT_EQ(AgentValues<std::vector<real_t>>::read(v, 1, 3).size(), 3u);

    EXPECT_EQ(observation_checksum(std::vector<real_t>{1.0, 2.0}),
              observation_checksum(std::vector<real_t>{1.0, 2.0}));
    EXPECT_NE(observation_checksum(std::vector<real_t>{1.0, 2.0}),
              observation_checksum(std::vector<real_t>{2.0, 1.0}));
    EXPECT_EQ(observation_checksum(0.0), observation_checksum(-0.0));
}

TEST(TestQLearningAgent, LearnsWhatTheTeamLearns)
{
    // a team of QLearningAgents, stepped by hand, against IndependentQLearning
    const uint_t N = 200;
    auto env = make_grid();
    IndependentQLearning team(env.n_agents(), env.n_states(), env.n_actions(), q_config(N));
    MultiAgentTrainer<MultiAgentGridWorld, IndependentQLearning> trainer({N}, team);
    trainer.train(env);

    auto env2 = make_grid();
    std::vector<QLearningAgent> agents;
    for (uint_t a = 0; a < env2.n_agents(); ++a)
    {
        agents.emplace_back(a, env2.n_states(), env2.n_actions(), q_config(N));
    }
    for (uint_t e = 0; e < N; ++e)
    {
        auto step = env2.reset();
        while (!step.last())
        {
            std::vector<uint_t> actions;
            for (auto &agent : agents)
            {
                actions.push_back(agent.act(step));
            }
            auto next = env2.step(actions);
            for (uint_t a = 0; a < agents.size(); ++a)
            {
                agents[a].update(step, actions[a], next);
            }
            step = std::move(next);
        }
        for (auto &agent : agents)
        {
            agent.on_episode_end(e);
        }
    }
    for (uint_t a = 0; a < agents.size(); ++a)
    {
        EXPECT_EQ(agents[a].q_table(), team.q_table(a));
    }
}

TEST(TestDistributedAgentEnv, LearnsWhatOneProcessLearns)
{
    const uint_t N = 300;
    auto env = make_grid();
    IndependentQLearning team(env.n_agents(), env.n_states(), env.n_actions(), q_config(N));
    MultiAgentTrainer<MultiAgentGridWorld, IndependentQLearning> trainer({N}, team);
    const auto central = trainer.train(env);

    InProcessBus bus;
    std::vector<std::unique_ptr<Transport>> transports;
    for (uint_t a = 0; a < env.n_agents(); ++a)
    {
        transports.push_back(bus.connect());
    }
    std::vector<MultiAgentTrainingResult> results;
    const auto agents = train_distributed(transports, N, results);
    for (uint_t a = 0; a < env.n_agents(); ++a)
    {
        EXPECT_EQ(agents[a].q_table(), team.q_table(a)) << "agent " << a;
        ASSERT_EQ(results[a].episodes.size(), N);
        for (uint_t e = 0; e < N; ++e)
        {
            EXPECT_EQ(results[a].episodes[e].returns, central.episodes[e].returns);
            EXPECT_EQ(results[a].episodes[e].n_steps, central.episodes[e].n_steps);
        }
    }
}

TEST(TestDistributedAgentEnv, DoesNotDependOnTheNetwork)
{
    const uint_t N = 8;
    InProcessBus clean_bus;
    std::vector<std::unique_ptr<Transport>> clean;
    InProcessBus lossy_bus;
    std::vector<std::unique_ptr<Transport>> lossy;
    LossyTransport::Config network;
    network.latency = 0.001;
    network.jitter = 0.001;
    network.loss = 0.2;
    for (uint_t a = 0; a < 2; ++a)
    {
        clean.push_back(clean_bus.connect());
        network.seed = a + 1;
        lossy.push_back(std::make_unique<LossyTransport>(lossy_bus.connect(), network));
    }
    std::vector<MultiAgentTrainingResult> r_clean, r_lossy;
    const auto a_clean = train_distributed(clean, N, r_clean);
    const auto a_lossy = train_distributed(lossy, N, r_lossy);
    for (uint_t a = 0; a < 2; ++a)
    {
        EXPECT_EQ(a_clean[a].q_table(), a_lossy[a].q_table());
    }
}

// Steer straight to the goal; the observation is [x, y, theta, goal_x - x, goal_y - y, ...]
struct ToGoal
{
    uint_t agent;
    std::array<real_t, 2> act(const MultiRobotNavigation::time_step_type &step) const
    {
        const auto &o = step.observation(agent);
        const real_t error = std::remainder(std::atan2(o[4], o[3]) - o[2], 2.0 * M_PI);
        return {0.5 * std::max(0.0, std::cos(error)), 3.0 * error};
    }
};

MultiRobotNavigation make_navigation(real_t start_noise, uint_t seed, uint_t n_robots = 3)
{
    MultiRobotNavigation env;
    env.make("v1",
             {{"n_robots", n_robots}, {"start_noise", start_noise}, {"max_steps", uint_t(60)}},
             {{"seed", seed}});
    return env;
}

TEST(TestDistributedAgentEnv, DrivesTheRobotsAsOneProcessDoes)
{
    // in one process
    auto env = make_navigation(0.05, 3);
    env.reset();
    auto step = env.reset();
    while (!step.last())
    {
        MultiRobotNavigation::joint_action_type actions;
        for (uint_t a = 0; a < env.n_agents(); ++a)
        {
            actions.push_back(ToGoal{a}.act(step));
        }
        step = env.step(actions);
    }

    // a thread per robot; every copy skips one episode as the one above did
    InProcessBus bus;
    std::vector<std::unique_ptr<Transport>> transports;
    for (uint_t a = 0; a < 3; ++a)
    {
        transports.push_back(bus.connect());
    }
    std::vector<std::vector<RobotPose>> poses(3);
    std::vector<std::thread> threads;
    for (uint_t a = 0; a < 3; ++a)
    {
        threads.emplace_back(
            [&, a]
            {
                auto copy = make_navigation(0.05, 3);
                copy.reset();
                DistributedAgentEnv<MultiRobotNavigation> mine(copy, a, *transports[a]);
                ToGoal actor{a};
                run_agent_episode(mine, actor);
                poses[a] = copy.poses();
            });
    }
    for (auto &t : threads)
    {
        t.join();
    }
    for (uint_t a = 0; a < 3; ++a)
    {
        ASSERT_EQ(poses[a].size(), 3u);
        for (uint_t r = 0; r < 3; ++r)
        {
            EXPECT_EQ(poses[a][r].x, env.poses()[r].x);
            EXPECT_EQ(poses[a][r].y, env.poses()[r].y);
            EXPECT_EQ(poses[a][r].theta, env.poses()[r].theta);
        }
    }
}

TEST(TestDistributedAgentEnv, TellsThatTheCopiesDiffer)
{
    // two robots whose copies start them elsewhere: other seeds
    InProcessBus bus;
    auto t0 = bus.connect();
    auto t1 = bus.connect();
    std::string error0, error1;
    auto run = [](Transport &t, uint_t agent, uint_t seed, std::string &error)
    {
        auto copy = make_navigation(0.05, seed, 2);
        DistributedAgentConfig config;
        config.timeout = 2.0;
        DistributedAgentEnv<MultiRobotNavigation> mine(copy, agent, t, config);
        try
        {
            auto step = mine.reset();
            mine.step(ToGoal{agent}.act(step));
        }
        catch (const std::runtime_error &e)
        {
            error = e.what();
        }
    };
    std::thread a([&] { run(*t0, 0, 1, error0); });
    std::thread b([&] { run(*t1, 1, 2, error1); });
    a.join();
    b.join();
    EXPECT_NE(error0.find("differ"), std::string::npos) << error0;
    EXPECT_NE(error1.find("differ"), std::string::npos) << error1;
}

TEST(TestDistributedAgentEnv, RejectsBadUse)
{
    InProcessBus bus;
    auto t = bus.connect();
    MultiAgentGridWorld unmade;
    EXPECT_THROW(DistributedAgentEnv<MultiAgentGridWorld>(unmade, 0, *t), std::logic_error);
    auto env = make_grid();
    EXPECT_THROW(DistributedAgentEnv<MultiAgentGridWorld>(env, 5, *t), std::invalid_argument);
    DistributedAgentEnv<MultiAgentGridWorld> mine(env, 0, *t);
    EXPECT_THROW(mine.step(0), std::logic_error);
}

} // namespace
