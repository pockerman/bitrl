#include "bitrl/envs/multi_agent/multi_robot_navigation.h"

#include "bitsim/bitsim_consts.h"
#include "bitsim/dynamics/diff_drive_dynamics.h"
#include "bitsim/dynamics/system_state.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>

namespace bitrl
{
namespace envs::multi_agent
{

namespace
{
real_t distance(real_t x0, real_t y0, real_t x1, real_t y1) { return std::hypot(x1 - x0, y1 - y0); }
} // namespace

MultiRobotNavigation::MultiRobotNavigation() : base_type("MultiRobotNavigation") {}

void MultiRobotNavigation::make(const std::string &version,
                                const std::unordered_map<std::string, std::any> &make_options,
                                const std::unordered_map<std::string, std::any> &reset_options)
{
    set_max_steps_(400);
    base_type::make(version, make_options, reset_options);

    const auto n_robots = option_<uint_t>("n_robots", 4);
    circle_radius_ = option_<real_t>("circle_radius", 2.0);
    robot_radius_ = option_<real_t>("robot_radius", 0.15);
    goal_tolerance_ = option_<real_t>("goal_tolerance", 0.1);
    dt_ = option_<real_t>("dt", 0.1);
    max_v_ = option_<real_t>("max_v", 0.5);
    max_w_ = option_<real_t>("max_w", 2.0);
    progress_weight_ = option_<real_t>("progress_weight", 1.0);
    time_reward_ = option_<real_t>("time_reward", -0.01);
    collision_reward_ = option_<real_t>("collision_reward", -1.0);
    goal_reward_ = option_<real_t>("goal_reward", 10.0);
    start_noise_ = option_<real_t>("start_noise", 0.0);

    if (n_robots < 1)
    {
        throw std::invalid_argument("MultiRobotNavigation: n_robots must be at least 1");
    }
    // neighbours on the circle must not overlap at the start
    const real_t gap = 2.0 * circle_radius_ * std::sin(bitsim::consts::maths::PI / n_robots);
    if (n_robots > 1 && gap <= 2.0 * robot_radius_)
    {
        throw std::invalid_argument("MultiRobotNavigation: the circle is too small for that many "
                                    "robots");
    }

    std::vector<std::string> names;
    for (uint_t i = 0; i < n_robots; ++i)
    {
        names.push_back("robot_" + std::to_string(i));
    }
    set_agents_(names);
    make_created_();
    reset(DEFAULT_ENV_SEED);
}

real_t MultiRobotNavigation::distance_to_goal(uint_t robot) const
{
    return distance(poses_.at(robot).x, poses_.at(robot).y, goals_.at(robot)[0],
                    goals_.at(robot)[1]);
}

real_t MultiRobotNavigation::closest_pair_() const
{
    real_t closest = std::numeric_limits<real_t>::max();
    for (uint_t i = 0; i < poses_.size(); ++i)
    {
        for (uint_t j = i + 1; j < poses_.size(); ++j)
        {
            closest =
                std::min(closest, distance(poses_[i].x, poses_[i].y, poses_[j].x, poses_[j].y));
        }
    }
    return closest;
}

std::vector<real_t> MultiRobotNavigation::observe_(uint_t robot) const
{
    const auto &p = poses_[robot];
    std::vector<real_t> observation{p.x, p.y, p.theta, goals_[robot][0] - p.x,
                                    goals_[robot][1] - p.y};
    observation.reserve(observation_size());
    for (uint_t j = 0; j < n_agents(); ++j)
    {
        if (j == robot)
        {
            continue;
        }
        observation.push_back(poses_[j].x - p.x);
        observation.push_back(poses_[j].y - p.y);
        observation.push_back(velocities_[j][0]);
        observation.push_back(velocities_[j][1]);
    }
    return observation;
}

MultiRobotNavigation::time_step_type
MultiRobotNavigation::make_time_step_(TimeStepTp type, std::vector<real_t> rewards, bool truncated,
                                      uint_t collisions) const
{
    std::vector<std::vector<real_t>> observations(n_agents());
    for (uint_t i = 0; i < n_agents(); ++i)
    {
        observations[i] = observe_(i);
    }
    return time_step_type(type, std::move(observations), std::move(rewards), dones_, truncated,
                          {{"collisions", std::any(collisions)}});
}

MultiRobotNavigation::time_step_type MultiRobotNavigation::reset(uint_t seed)
{
    if (!is_created())
    {
        throw std::logic_error("MultiRobotNavigation: call make() before reset()");
    }
    const uint_t n = n_agents();
    std::mt19937 rng(static_cast<std::mt19937::result_type>(seed));
    std::normal_distribution<real_t> noise(0.0, start_noise_ > 0.0 ? start_noise_ : 1.0);

    poses_.assign(n, RobotPose{});
    goals_.assign(n, {0.0, 0.0});
    velocities_.assign(n, {0.0, 0.0});
    dones_.assign(n, false);
    for (uint_t i = 0; i < n; ++i)
    {
        const real_t angle = 2.0 * bitsim::consts::maths::PI * i / n;
        poses_[i].x = circle_radius_ * std::cos(angle);
        poses_[i].y = circle_radius_ * std::sin(angle);
        if (start_noise_ > 0.0)
        {
            poses_[i].x += noise(rng);
            poses_[i].y += noise(rng);
        }
        poses_[i].theta = std::atan2(-poses_[i].y, -poses_[i].x);
        goals_[i] = {-circle_radius_ * std::cos(angle), -circle_radius_ * std::sin(angle)};
    }
    n_collisions_ = 0;
    min_separation_ = n > 1 ? closest_pair_() : 0.0;
    start_episode_();
    return make_time_step_(TimeStepTp::FIRST, std::vector<real_t>(n, 0.0), false, 0);
}

MultiRobotNavigation::time_step_type MultiRobotNavigation::step(const joint_action_type &actions)
{
    check_step_(actions);
    const uint_t n = n_agents();

    // move every robot that is not done, with the kinematics of bitsim's differential drive
    std::vector<RobotPose> proposed = poses_;
    std::vector<real_t> speeds(n, 0.0);
    for (uint_t i = 0; i < n; ++i)
    {
        if (dones_[i])
        {
            continue;
        }
        speeds[i] = std::clamp(actions[i][0], -max_v_, max_v_);
        const real_t w = std::clamp(actions[i][1], -max_w_, max_w_);
        bitsim::dynamics::SysState<3> state({std::make_pair("X", poses_[i].x),
                                             std::make_pair("Y", poses_[i].y),
                                             std::make_pair("Theta", poses_[i].theta)});
        const auto next = bitsim::dynamics::DiffDriveDynamics::integrate_state_v2(
            state, dt_, speeds[i], w, {0.0, 0.0});
        proposed[i] = {next[0], next[1], std::remainder(next[2], 2.0 * bitsim::consts::maths::PI)};
    }

    // cancel the moves that would make two robots overlap, until none does; a cancelled robot
    // still turns, so that it can steer away
    const real_t contact = 2.0 * robot_radius_;
    std::vector<bool> collided(n, false);
    bool changed = true;
    while (changed)
    {
        changed = false;
        for (uint_t i = 0; i < n; ++i)
        {
            for (uint_t j = i + 1; j < n; ++j)
            {
                const real_t after =
                    distance(proposed[i].x, proposed[i].y, proposed[j].x, proposed[j].y);
                const real_t before = distance(poses_[i].x, poses_[i].y, poses_[j].x, poses_[j].y);
                // robots that already touch may move apart
                if (after >= contact || after >= before)
                {
                    continue;
                }
                for (const uint_t k : {i, j})
                {
                    if (proposed[k].x != poses_[k].x || proposed[k].y != poses_[k].y)
                    {
                        proposed[k].x = poses_[k].x;
                        proposed[k].y = poses_[k].y;
                        collided[k] = true;
                        changed = true;
                    }
                }
            }
        }
    }

    std::vector<real_t> rewards(n, 0.0);
    uint_t collisions = 0;
    for (uint_t i = 0; i < n; ++i)
    {
        if (dones_[i])
        {
            velocities_[i] = {0.0, 0.0};
            continue;
        }
        const real_t before = distance_to_goal(i);
        velocities_[i] = {(proposed[i].x - poses_[i].x) / dt_, (proposed[i].y - poses_[i].y) / dt_};
        poses_[i] = proposed[i];
        const real_t after = distance_to_goal(i);

        rewards[i] = progress_weight_ * (before - after) + time_reward_;
        if (collided[i])
        {
            rewards[i] += collision_reward_;
            ++collisions;
        }
        if (after <= goal_tolerance_)
        {
            rewards[i] += goal_reward_;
            dones_[i] = true;
            velocities_[i] = {0.0, 0.0};
        }
    }
    n_collisions_ += collisions;
    if (n > 1)
    {
        min_separation_ = std::min(min_separation_, closest_pair_());
    }

    const bool all_done = std::all_of(dones_.begin(), dones_.end(), [](bool d) { return d; });
    const bool out_of_steps = count_step_();
    const bool last = all_done || out_of_steps;
    return make_time_step_(last ? TimeStepTp::LAST : TimeStepTp::MID, std::move(rewards),
                           out_of_steps && !all_done, collisions);
}

} // namespace envs::multi_agent
} // namespace bitrl
