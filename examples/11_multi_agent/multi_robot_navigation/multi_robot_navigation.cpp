// Differential drive robots cross a circle to the opposite point, all at once, each with its own
// controller that sees only its own observation. A controller that drives straight to the goal
// jams in the middle; one that also steers around the robots it sees gets everyone through.
// A sweep over the number of robots then runs many simulations at the same time, with bitsim's
// parallel_map.
//
// Usage: multi_robot_navigation [--robots N]
//
// Writes trajectories.csv (every robot at every step, for the avoiding controllers) to the working
// directory. examples/tools/multi_agent_gif.py turns it into a GIF.

#include "bitrl/envs/multi_agent/multi_robot_navigation.h"
#include "bitrl/base/bitrl_types.h"
#include "bitrl/rl/multi_agent/multi_agent_trainer.h"
#include "bitsim/bitsim_consts.h"
#include "bitsim/utils/concurrency/parallel.h"
#include "bitsim/utils/io/print_utils.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace multi_robot_navigation
{

using bitrl::real_t;
using bitrl::uint_t;
using namespace bitrl::envs::multi_agent;
using namespace bitrl::rl::multi_agent;

typedef MultiRobotNavigation::time_step_type time_step_type;
typedef MultiRobotNavigation::joint_action_type joint_action_type;
typedef std::array<real_t, 2> action_type;

const real_t PI = bitsim::consts::maths::PI;

real_t wrap(real_t angle) { return std::remainder(angle, 2.0 * PI); }

// Drive towards the goal: turn to face it, and go slower near it. The observation is
// [x, y, theta, goal_x - x, goal_y - y, then x_j - x, y_j - y, vx_j, vy_j for every other robot]
action_type go_to_goal(const std::vector<real_t> &o, real_t max_v, real_t max_w)
{
    const real_t distance = std::hypot(o[3], o[4]);
    const real_t error = wrap(std::atan2(o[4], o[3]) - o[2]);
    const real_t v = std::min(max_v, distance) * std::max(0.0, std::cos(error));
    const real_t w = std::clamp(3.0 * error, -max_w, max_w);
    return {v, w};
}

// Drive towards the goal, but push the heading away from the robots within SENSE metres, and a
// little to the right of them, so that two robots that meet head on pass each other on the same
// side instead of stopping face to face. Slow down when a robot is close ahead.
action_type avoid(const std::vector<real_t> &o, real_t max_v, real_t max_w, real_t robot_radius)
{
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
        // grows from 0 at SENSE to large at contact
        const real_t gap = std::max(r - 2.0 * robot_radius, 0.05);
        const real_t push = (1.0 / gap - 1.0 / SENSE);
        dx -= push * rx / r * 0.3;
        dy -= push * ry / r * 0.3;
        // the right-hand side of the direction to the other robot
        dx += push * ry / r * 0.3;
        dy -= push * rx / r * 0.3;
        // a robot ahead of us slows us down
        const real_t ahead = (rx * std::cos(o[2]) + ry * std::sin(o[2])) / r;
        if (ahead > 0.0)
        {
            slow = std::min(slow, std::clamp((r - 2.0 * robot_radius) / (SENSE * 0.5), 0.0, 1.0));
        }
    }
    const real_t error = wrap(std::atan2(dy, dx) - o[2]);
    const real_t v =
        std::min(max_v, distance) * std::max(0.0, std::cos(error)) * std::max(slow, 0.2);
    const real_t w = std::clamp(3.0 * error, -max_w, max_w);
    return {v, w};
}

// A team of robots, each with its own controller: the controller of robot i sees only the
// observation of robot i, so the team is decentralised
struct Team
{
    const MultiRobotNavigation &env;
    bool avoiding;

    joint_action_type act(const time_step_type &step) const
    {
        joint_action_type actions(step.n_agents());
        for (uint_t i = 0; i < step.n_agents(); ++i)
        {
            const auto &o = step.observation(i);
            actions[i] = avoiding ? avoid(o, env.max_v(), env.max_w(), env.robot_radius())
                                  : go_to_goal(o, env.max_v(), env.max_w());
        }
        return actions;
    }
};

MultiRobotNavigation make_env(uint_t n_robots, real_t start_noise, uint_t seed)
{
    MultiRobotNavigation env;
    env.make("v1",
             {{"n_robots", n_robots},
              {"circle_radius", 2.0},
              {"start_noise", start_noise},
              {"max_steps", uint_t(600)}},
             {{"seed", seed}});
    return env;
}

void report(const std::string &name, const MultiRobotNavigation &env,
            const MultiAgentEpisodeResult &result)
{
    uint_t arrived = 0;
    for (uint_t i = 0; i < env.n_agents(); ++i)
    {
        arrived += env.distance_to_goal(i) <= 0.1 ? 1 : 0;
    }
    std::cout << "  " << name << ": " << arrived << " of " << env.n_agents()
              << " robots arrived in " << std::fixed << std::setprecision(1)
              << result.n_steps * env.dt() << " s, " << result.n_collisions
              << " moves blocked by a collision, closest distance between two centres "
              << std::setprecision(2) << env.min_separation() << " m\n";
}

void run(uint_t n_robots)
{
    bitsim::utils::io::print_title("1. Straight to the goal");
    {
        auto env = make_env(n_robots, 0.0, 1);
        Team team{env, false};
        const auto result = run_episode(env, team);
        report("go to goal", env, result);
    }
    std::cout << "\n";

    bitsim::utils::io::print_title("2. Steering around the others");
    {
        auto env = make_env(n_robots, 0.0, 1);
        Team team{env, true};
        std::ofstream out("trajectories.csv");
        out << "step,robot,x,y,theta,goal_x,goal_y,radius\n";
        uint_t k = 0;
        const auto result = run_episode(env, team,
                                        [&](const time_step_type &)
                                        {
                                            for (uint_t i = 0; i < env.n_agents(); ++i)
                                            {
                                                const auto &p = env.poses()[i];
                                                out << k << "," << i << "," << p.x << "," << p.y
                                                    << "," << p.theta << "," << env.goals()[i][0]
                                                    << "," << env.goals()[i][1] << ","
                                                    << env.robot_radius() << "\n";
                                            }
                                            ++k;
                                        });
        report("avoiding", env, result);
        std::cout << "  The paths are in trajectories.csv\n\n";
    }

    bitsim::utils::io::print_title("3. More robots, in parallel");
    // 10 noisy starts for every number of robots and both controllers; every simulation is on its
    // own, so parallel_map runs them on all the threads and returns them in order
    const std::vector<uint_t> sizes{2, 4, 6, 8, 10, 12};
    const uint_t N_STARTS = 10;
    struct Outcome
    {
        bool all_arrived;
        real_t time;
        uint_t collisions;
    };
    const auto outcomes = bitsim::utils::concurrency::parallel_map(
        sizes.size() * 2 * N_STARTS,
        [&](std::size_t k)
        {
            const uint_t n = sizes[k / (2 * N_STARTS)];
            const bool avoiding = (k / N_STARTS) % 2 == 1;
            auto env = make_env(n, 0.05, bitsim::utils::concurrency::task_seed(7, k % N_STARTS));
            Team team{env, avoiding};
            const auto result = run_episode(env, team);
            return Outcome{result.all_done, result.n_steps * env.dt(), result.n_collisions};
        });

    std::cout << "  robots   controller   all arrived   mean time [s]   blocked moves\n";
    for (uint_t s = 0; s < sizes.size(); ++s)
    {
        for (uint_t c = 0; c < 2; ++c)
        {
            uint_t arrived = 0, collisions = 0;
            real_t time = 0.0;
            for (uint_t r = 0; r < N_STARTS; ++r)
            {
                const auto &o = outcomes[(s * 2 + c) * N_STARTS + r];
                arrived += o.all_arrived ? 1 : 0;
                time += o.all_arrived ? o.time : 0.0;
                collisions += o.collisions;
            }
            std::ostringstream mean_time;
            mean_time << std::fixed << std::setprecision(1);
            if (arrived)
            {
                mean_time << time / arrived;
            }
            else
            {
                mean_time << "-";
            }
            std::cout << std::setw(8) << sizes[s] << std::setw(13) << (c ? "avoiding" : "to goal")
                      << std::setw(11) << arrived << "/" << N_STARTS << std::setw(16)
                      << mean_time.str() << std::setw(16) << std::setprecision(1)
                      << static_cast<real_t>(collisions) / N_STARTS << "\n";
        }
    }
}

} // namespace multi_robot_navigation

int main(int argc, char **argv)
{
    bitrl::uint_t n_robots = 6;
    for (int i = 1; i + 1 < argc; i += 2)
    {
        if (std::string(argv[i]) == "--robots")
        {
            n_robots = std::strtoul(argv[i + 1], nullptr, 10);
        }
    }
    multi_robot_navigation::run(n_robots);
    return 0;
}
