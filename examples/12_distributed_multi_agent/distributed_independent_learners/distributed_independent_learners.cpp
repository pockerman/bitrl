// Two agents learn to cross a corridor with independent Q-learning, each in a participant of its
// own: a thread, a process or a machine. Every participant holds a copy of the grid world and
// learns only its own agent's Q-table; at every step the participants send each other their
// actions, and step their copies with the joint action. The copies stay alike, so the agents
// learn exactly what they learn in one process.
//
// Usage: distributed_independent_learners [--episodes N]
//        distributed_independent_learners --transport mqtt --agent 0|1 [--episodes N] [--broker
//        URL]
//
// In-process (the default) the example trains the team in one process, then with a thread per
// agent, and compares. With --transport mqtt every agent is a process of its own; start one with
// --agent 0 and one with --agent 1, on the same broker. Each also trains the team in one process,
// to tell whether it learned the same.

#include "bitrl/base/bitrl_types.h"
#include "bitrl/envs/multi_agent/distributed_agent_env.h"
#include "bitrl/envs/multi_agent/multi_agent_grid_world.h"
#include "bitrl/rl/multi_agent/agent_trainer.h"
#include "bitrl/rl/multi_agent/independent_q_learning.h"
#include "bitrl/rl/multi_agent/multi_agent_trainer.h"
#include "bitrl/rl/multi_agent/q_learning_agent.h"
#include "bitsim/bitsim_config.h"
#include "bitsim/distributed/transport.h"
#include "bitsim/utils/concurrency/parallel.h"
#include "bitsim/utils/io/print_utils.h"

#ifdef BITSIM_MQTT
#include "bitsim/distributed/mqtt_transport.h"
#endif

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace distributed_independent_learners
{

using bitrl::DynMat;
using bitrl::real_t;
using bitrl::uint_t;
using namespace bitrl::envs::multi_agent;
using namespace bitrl::rl::multi_agent;
using bitsim::distributed::InProcessBus;
using bitsim::distributed::Transport;
using steady = std::chrono::steady_clock;

typedef DistributedAgentEnv<MultiAgentGridWorld> agent_env_type;

// every participant makes its copy with the same options, and the same seed
MultiAgentGridWorld make_env()
{
    MultiAgentGridWorld env;
    env.make("v1", {{"observation", GridObservation::JOINT_POSITIONS}, {"shared_reward", true}},
             {{"seed", uint_t(1)}});
    return env;
}

IndependentQLearningConfig learning(uint_t n_episodes)
{
    IndependentQLearningConfig config;
    config.seed = bitsim::utils::concurrency::task_seed(1, 0);
    config.eta = 0.2;
    config.eps_decay = 5.0 / n_episodes;
    return config;
}

real_t seconds(steady::duration d) { return std::chrono::duration<real_t>(d).count(); }

// What one participant learned, and how its greedy agent did with the others
struct Outcome
{
    MultiAgentTrainingResult training;
    MultiAgentEpisodeResult greedy;
    DynMat<real_t> q_table;
    uint_t n_resent{0};
};

// The participant of one agent: train it, then play a greedy episode with the others
Outcome participant(uint_t agent, Transport &transport, uint_t n_episodes)
{
    auto copy = make_env();
    QLearningAgent learner(agent, copy.n_states(), copy.n_actions(), learning(n_episodes));
    agent_env_type env(copy, agent, transport);

    AgentTrainer<agent_env_type, QLearningAgent> trainer({n_episodes}, learner);
    Outcome outcome;
    outcome.training = trainer.train(env);
    auto greedy = learner.greedy();
    outcome.greedy = run_agent_episode(env, greedy);
    outcome.q_table = learner.q_table();
    outcome.n_resent = env.synchroniser().n_resent();
    env.leave();
    return outcome;
}

// The same team in one process
struct Team
{
    MultiAgentTrainingResult training;
    MultiAgentEpisodeResult greedy;
    std::vector<DynMat<real_t>> q_tables;
};

Team one_process(uint_t n_episodes)
{
    auto env = make_env();
    IndependentQLearning team(env.n_agents(), env.n_states(), env.n_actions(),
                              learning(n_episodes));
    MultiAgentTrainer<MultiAgentGridWorld, IndependentQLearning> trainer({n_episodes}, team);
    Team result;
    result.training = trainer.train(env);
    auto greedy = team.greedy();
    result.greedy = run_episode(env, greedy);
    for (uint_t a = 0; a < env.n_agents(); ++a)
    {
        result.q_tables.push_back(team.q_table(a));
    }
    return result;
}

uint_t total_steps(const MultiAgentTrainingResult &training)
{
    uint_t n = 0;
    for (const auto &e : training.episodes)
    {
        n += e.n_steps;
    }
    return n;
}

bool same_returns(const MultiAgentTrainingResult &a, const MultiAgentTrainingResult &b)
{
    if (a.episodes.size() != b.episodes.size())
    {
        return false;
    }
    for (std::size_t e = 0; e < a.episodes.size(); ++e)
    {
        if (a.episodes[e].returns != b.episodes[e].returns)
        {
            return false;
        }
    }
    return true;
}

void report_greedy(const MultiAgentEpisodeResult &greedy)
{
    std::cout << "  the greedy team " << (greedy.all_done ? "crossed" : "did not cross") << " in "
              << greedy.n_steps << " steps, team return " << std::fixed << std::setprecision(2)
              << greedy.team_return() << "\n";
}

int run_in_process(uint_t n_episodes)
{
    bitsim::utils::io::print_title("1. The team in one process");
    auto start = steady::now();
    const auto team = one_process(n_episodes);
    const real_t t_one = seconds(steady::now() - start);
    const uint_t n_steps = total_steps(team.training);
    std::cout << "  " << n_episodes << " episodes, " << n_steps << " steps, in " << std::fixed
              << std::setprecision(2) << t_one << " s\n";
    report_greedy(team.greedy);
    std::cout << "\n";

    bitsim::utils::io::print_title("2. A participant per agent");
    InProcessBus bus;
    const uint_t n = make_env().n_agents();
    std::vector<std::unique_ptr<Transport>> transports;
    for (uint_t a = 0; a < n; ++a)
    {
        transports.push_back(bus.connect());
    }
    std::vector<Outcome> outcomes(n);
    start = steady::now();
    std::vector<std::thread> threads;
    for (uint_t a = 0; a < n; ++a)
    {
        threads.emplace_back([&, a] { outcomes[a] = participant(a, *transports[a], n_episodes); });
    }
    for (auto &t : threads)
    {
        t.join();
    }
    const real_t t_distributed = seconds(steady::now() - start);
    std::cout << "  " << n << " threads, one per agent, in " << std::setprecision(2)
              << t_distributed << " s: " << std::setprecision(1) << 1e6 * t_distributed / n_steps
              << " microseconds per step for the exchange\n";
    report_greedy(outcomes.front().greedy);

    bool same = true;
    for (uint_t a = 0; a < n; ++a)
    {
        same = same && outcomes[a].q_table == team.q_tables[a] &&
               same_returns(outcomes[a].training, team.training);
    }
    std::cout << "  every agent learned the same Q-table, and every episode had the same "
              << "returns, as in one process: " << std::boolalpha << same << "\n";
    return same ? 0 : 1;
}

int run_mqtt(uint_t agent, uint_t n_episodes, const std::string &broker)
{
#ifdef BITSIM_MQTT
    if (agent >= make_env().n_agents())
    {
        std::cerr << "--agent must be 0 or 1\n";
        return 1;
    }
    bitsim::distributed::MQTT_Transport transport(broker, "distributed_independent_learners");
    std::cout << "  agent " << make_env().agent_name(agent) << ": waiting for the others on "
              << broker << "\n";
    const auto start = steady::now();
    const auto outcome = participant(agent, transport, n_episodes);
    const real_t t = seconds(steady::now() - start);
    std::cout << "  " << n_episodes << " episodes in " << std::fixed << std::setprecision(2) << t
              << " s, " << std::setprecision(1) << 1e6 * t / total_steps(outcome.training)
              << " microseconds per step; sent its last steps again " << outcome.n_resent
              << " times\n";
    report_greedy(outcome.greedy);
    const auto team = one_process(n_episodes);
    std::cout << "  learned the same Q-table as the team in one process: " << std::boolalpha
              << (outcome.q_table == team.q_tables[agent]) << "\n";
    return 0;
#else
    (void)agent;
    (void)n_episodes;
    (void)broker;
    std::cerr << "bitsim was built without MQTT\n";
    return 1;
#endif
}

} // namespace distributed_independent_learners

int main(int argc, char **argv)
{
    bitrl::uint_t n_episodes = 4000;
    bitrl::uint_t agent = 0;
    std::string transport = "inprocess", broker = "tcp://localhost:1883";
    for (int i = 1; i + 1 < argc; i += 2)
    {
        const std::string flag = argv[i], value = argv[i + 1];
        if (flag == "--episodes")
            n_episodes = std::strtoul(value.c_str(), nullptr, 10);
        else if (flag == "--transport")
            transport = value;
        else if (flag == "--agent")
            agent = std::strtoul(value.c_str(), nullptr, 10);
        else if (flag == "--broker")
            broker = value;
        else
        {
            std::cerr << "unknown option " << flag << "\n";
            return 1;
        }
    }
    if (transport == "mqtt")
    {
        return distributed_independent_learners::run_mqtt(agent, n_episodes, broker);
    }
    return distributed_independent_learners::run_in_process(n_episodes);
}
