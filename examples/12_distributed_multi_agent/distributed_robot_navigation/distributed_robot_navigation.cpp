// Differential drive robots cross a circle to the opposite point, each driven by a controller in a
// participant of its own, as the on-board computer of every robot would be. Every participant
// holds a copy of MultiRobotNavigation; at every step the controllers send each other their
// velocity commands and step their copies with all of them, so every copy moves every robot
// alike, and the robots drive exactly as they do in one process.
//
// Usage: distributed_robot_navigation [--robots N]
//        distributed_robot_navigation --transport mqtt --robot K [--robots N] [--broker URL]
//
// In-process (the default) the example drives the robots in one process, then with a thread per
// robot, and compares. With --transport mqtt every robot is a process of its own: start one for
// every K from 0 to N - 1, on the same broker. Writes trajectories_<K>.csv, the copy's paths of
// every robot, in MQTT mode.

#include "bitrl/base/bitrl_types.h"
#include "bitrl/envs/multi_agent/distributed_agent_env.h"
#include "bitrl/envs/multi_agent/multi_robot_navigation.h"
#include "bitrl/rl/multi_agent/agent_trainer.h"
#include "bitrl/rl/multi_agent/multi_agent_trainer.h"
#include "bitsim/bitsim_config.h"
#include "bitsim/bitsim_consts.h"
#include "bitsim/distributed/transport.h"
#include "bitsim/utils/io/print_utils.h"

#ifdef BITSIM_MQTT
#include "bitsim/distributed/mqtt_transport.h"
#endif

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace distributed_robot_navigation
{

using bitrl::real_t;
using bitrl::uint_t;
using namespace bitrl::envs::multi_agent;
using namespace bitrl::rl::multi_agent;
using bitsim::distributed::InProcessBus;
using bitsim::distributed::Transport;
using steady = std::chrono::steady_clock;

typedef MultiRobotNavigation::time_step_type time_step_type;
typedef std::array<real_t, 2> action_type;
typedef DistributedAgentEnv<MultiRobotNavigation> agent_env_type;

const real_t PI = bitsim::consts::maths::PI;

// The controller of one robot: drive to the goal, but push the heading away from the robots
// within a metre, and a little to the right of them, so that two that meet head on pass on the
// same side; slow down when one is close ahead. It reads only its own robot's observation,
// [x, y, theta, goal_x - x, goal_y - y, then x_j - x, y_j - y, vx_j, vy_j for every other robot]
struct Controller
{
    uint_t robot;
    real_t max_v, max_w, robot_radius;

    action_type act(const time_step_type &step) const
    {
        const auto &o = step.observation(robot);
        const real_t SENSE = 1.0;
        const real_t distance = std::hypot(o[3], o[4]);
        real_t dx = o[3] / std::max(distance, 1e-9);
        real_t dy = o[4] / std::max(distance, 1e-9);
        real_t slow = 1.0;
        for (std::size_t j = 5; j + 3 < o.size(); j += 4)
        {
            const real_t rx = o[j], ry = o[j + 1];
            const real_t r = std::hypot(rx, ry);
            if (r > SENSE || r < 1e-9)
            {
                continue;
            }
            const real_t push = 1.0 / std::max(r - 2.0 * robot_radius, 0.05) - 1.0 / SENSE;
            dx += 0.3 * push * (ry - rx) / r;
            dy -= 0.3 * push * (ry + rx) / r;
            if (rx * std::cos(o[2]) + ry * std::sin(o[2]) > 0.0)
            {
                slow =
                    std::min(slow, std::clamp((r - 2.0 * robot_radius) / (0.5 * SENSE), 0.0, 1.0));
            }
        }
        const real_t error = std::remainder(std::atan2(dy, dx) - o[2], 2.0 * PI);
        const real_t v =
            std::min(max_v, distance) * std::max(0.0, std::cos(error)) * std::max(slow, 0.2);
        return {v, std::clamp(3.0 * error, -max_w, max_w)};
    }
};

// every participant makes its copy with the same options
MultiRobotNavigation make_env(uint_t n_robots)
{
    MultiRobotNavigation env;
    env.make("v1",
             {{"n_robots", n_robots},
              {"circle_radius", 2.0},
              {"start_noise", 0.05},
              {"max_steps", uint_t(600)}},
             {{"seed", uint_t(3)}});
    return env;
}

Controller controller(const MultiRobotNavigation &env, uint_t robot)
{
    return {robot, env.max_v(), env.max_w(), env.robot_radius()};
}

// Every controller in one process, for the comparison
struct Team
{
    const MultiRobotNavigation &env;
    MultiRobotNavigation::joint_action_type act(const time_step_type &step) const
    {
        MultiRobotNavigation::joint_action_type actions;
        for (uint_t r = 0; r < env.n_agents(); ++r)
        {
            actions.push_back(controller(env, r).act(step));
        }
        return actions;
    }
};

real_t seconds(steady::duration d) { return std::chrono::duration<real_t>(d).count(); }

// The paths of every robot, as one copy saw them
typedef std::vector<std::vector<RobotPose>> Paths;

// The participant of one robot
struct Outcome
{
    MultiAgentEpisodeResult result;
    Paths paths;
    real_t min_separation{0.0};
};

Outcome participant(uint_t robot, uint_t n_robots, Transport &transport)
{
    auto copy = make_env(n_robots);
    agent_env_type env(copy, robot, transport);
    const auto me = controller(copy, robot);
    Outcome outcome;
    outcome.result = run_agent_episode(
        env, me, [&](const time_step_type &) { outcome.paths.push_back(copy.poses()); });
    outcome.min_separation = copy.min_separation();
    env.leave();
    return outcome;
}

void report(const MultiRobotNavigation &env, const MultiAgentEpisodeResult &result,
            real_t min_separation)
{
    std::cout << "  " << (result.all_done ? "every robot arrived" : "not every robot arrived")
              << " after " << std::fixed << std::setprecision(1) << result.n_steps * env.dt()
              << " s of simulated time; " << result.n_collisions
              << " moves blocked; closest centres " << std::setprecision(2) << min_separation
              << " m\n";
}

int run_in_process(uint_t n_robots)
{
    bitsim::utils::io::print_title("1. Every controller in one process");
    auto env = make_env(n_robots);
    Paths paths;
    auto start = steady::now();
    const Team team{env};
    const auto one =
        run_episode(env, team, [&](const time_step_type &) { paths.push_back(env.poses()); });
    const real_t t_one = seconds(steady::now() - start);
    report(env, one, env.min_separation());
    std::cout << "  in " << std::setprecision(4) << t_one << " s\n\n";

    bitsim::utils::io::print_title("2. A participant per robot");
    InProcessBus bus;
    std::vector<std::unique_ptr<Transport>> transports;
    for (uint_t r = 0; r < n_robots; ++r)
    {
        transports.push_back(bus.connect());
    }
    std::vector<Outcome> outcomes(n_robots);
    start = steady::now();
    std::vector<std::thread> threads;
    for (uint_t r = 0; r < n_robots; ++r)
    {
        threads.emplace_back([&, r] { outcomes[r] = participant(r, n_robots, *transports[r]); });
    }
    for (auto &t : threads)
    {
        t.join();
    }
    const real_t t_distributed = seconds(steady::now() - start);
    report(env, outcomes.front().result, outcomes.front().min_separation);
    std::cout << "  " << n_robots << " threads in " << std::setprecision(4) << t_distributed
              << " s, " << std::setprecision(1) << 1e6 * t_distributed / one.n_steps
              << " microseconds per step\n";

    bool same = true;
    for (const auto &o : outcomes)
    {
        same = same && o.paths.size() == paths.size();
        for (std::size_t k = 0; same && k < paths.size(); ++k)
        {
            for (uint_t r = 0; r < n_robots; ++r)
            {
                same = same && o.paths[k][r].x == paths[k][r].x &&
                       o.paths[k][r].y == paths[k][r].y && o.paths[k][r].theta == paths[k][r].theta;
            }
        }
    }
    std::cout << "  every copy moved every robot as the one process did: " << std::boolalpha << same
              << "\n";
    return same ? 0 : 1;
}

int run_mqtt(uint_t robot, uint_t n_robots, const std::string &broker)
{
#ifdef BITSIM_MQTT
    if (robot >= n_robots)
    {
        std::cerr << "--robot must be less than --robots\n";
        return 1;
    }
    bitsim::distributed::MQTT_Transport transport(broker, "distributed_robot_navigation");
    const auto env = make_env(n_robots);
    std::cout << "  " << env.agent_name(robot) << ": waiting for the others on " << broker << "\n";
    const auto start = steady::now();
    const auto outcome = participant(robot, n_robots, transport);
    const real_t t = seconds(steady::now() - start);
    report(env, outcome.result, outcome.min_separation);
    std::cout << "  in " << std::setprecision(2) << t << " s\n";

    std::ofstream out("trajectories_" + std::to_string(robot) + ".csv");
    out << std::setprecision(17) << "step,robot,x,y,theta\n";
    for (std::size_t k = 0; k < outcome.paths.size(); ++k)
    {
        for (uint_t r = 0; r < n_robots; ++r)
        {
            const auto &p = outcome.paths[k][r];
            out << k << "," << r << "," << p.x << "," << p.y << "," << p.theta << "\n";
        }
    }
    return 0;
#else
    (void)robot;
    (void)n_robots;
    (void)broker;
    std::cerr << "bitsim was built without MQTT\n";
    return 1;
#endif
}

} // namespace distributed_robot_navigation

int main(int argc, char **argv)
{
    bitrl::uint_t n_robots = 6;
    bitrl::uint_t robot = 0;
    std::string transport = "inprocess", broker = "tcp://localhost:1883";
    for (int i = 1; i + 1 < argc; i += 2)
    {
        const std::string flag = argv[i], value = argv[i + 1];
        if (flag == "--robots")
            n_robots = std::strtoul(value.c_str(), nullptr, 10);
        else if (flag == "--transport")
            transport = value;
        else if (flag == "--robot")
            robot = std::strtoul(value.c_str(), nullptr, 10);
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
        return distributed_robot_navigation::run_mqtt(robot, n_robots, broker);
    }
    return distributed_robot_navigation::run_in_process(n_robots);
}
