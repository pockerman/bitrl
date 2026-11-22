#ifndef MULTI_ROBOT_NAVIGATION_H
#define MULTI_ROBOT_NAVIGATION_H

#include "bitrl/base/bitrl_types.h"
#include "bitrl/envs/multi_agent/multi_agent_env_base.h"

#include <any>
#include <array>
#include <string>
#include <unordered_map>
#include <vector>

namespace bitrl
{
namespace envs::multi_agent
{

///
/// \brief The pose of a robot of MultiRobotNavigation: position and heading
///
struct RobotPose
{
    real_t x{0.0};
    real_t y{0.0};
    real_t theta{0.0};
};

///
/// \brief Differential drive robots that cross a circle to the opposite point, all at the same
/// time, without driving into each other.
///
/// The robots start evenly spaced on a circle, facing its centre, and each has to reach the point
/// of the circle opposite its start, so that all of them meet in the middle. A robot moves with
/// the kinematics of bitsim::dynamics::DiffDriveDynamics: its action is a linear and an angular
/// velocity, {v, w}, clipped to max_v and max_w. The robots are discs of radius robot_radius; a
/// move that would make two discs overlap is cancelled, the robot stands still for that step and
/// the move counts as a collision. A robot within goal_tolerance of its goal is done and stops.
///
/// The observation of robot i, all in the world frame, is
/// [x, y, theta, goal_x - x, goal_y - y, then for every other robot j in order:
/// x_j - x, y_j - y, vx_j, vy_j], so 5 + 4 (n_robots - 1) numbers.
///
/// The reward of a robot that is not done is progress_weight times the distance it gained to its
/// goal, plus time_reward, plus collision_reward on a collision and goal_reward when it arrives.
///
/// Make options, all optional: "n_robots" (uint_t, 4), "circle_radius" (real_t, 2.0),
/// "robot_radius" (real_t, 0.15), "goal_tolerance" (real_t, 0.1), "dt" (real_t, 0.1),
/// "max_v" (real_t, 0.5), "max_w" (real_t, 2.0), "progress_weight" (real_t, 1.0),
/// "time_reward" (real_t, -0.01), "collision_reward" (real_t, -1.0), "goal_reward" (real_t, 10.0),
/// "start_noise" (real_t, 0.0: the standard deviation of a random shift of the starts, drawn
/// with the seed of the reset), "max_steps" (uint_t, 400).
///
class MultiRobotNavigation final
    : public MultiAgentEnvBase<std::vector<real_t>, std::array<real_t, 2>>
{
  public:
    typedef MultiAgentEnvBase<std::vector<real_t>, std::array<real_t, 2>> base_type;
    typedef typename base_type::time_step_type time_step_type;
    typedef typename base_type::joint_action_type joint_action_type;

    MultiRobotNavigation();

    void make(const std::string &version,
              const std::unordered_map<std::string, std::any> &make_options,
              const std::unordered_map<std::string, std::any> &reset_options) override;

    using base_type::reset;

    time_step_type reset(uint_t seed) override;

    time_step_type step(const joint_action_type &actions) override;

    ///
    /// \brief The size of the observation of one robot
    ///
    uint_t observation_size() const noexcept { return 5 + 4 * (n_agents() - 1); }

    const std::vector<RobotPose> &poses() const noexcept { return poses_; }

    const std::vector<std::array<real_t, 2>> &goals() const noexcept { return goals_; }

    ///
    /// \brief The velocity {vx, vy} of every robot in the last step
    ///
    const std::vector<std::array<real_t, 2>> &velocities() const noexcept { return velocities_; }

    real_t robot_radius() const noexcept { return robot_radius_; }

    real_t circle_radius() const noexcept { return circle_radius_; }

    real_t dt() const noexcept { return dt_; }

    real_t max_v() const noexcept { return max_v_; }

    real_t max_w() const noexcept { return max_w_; }

    ///
    /// \brief The distance of robot to its goal
    ///
    real_t distance_to_goal(uint_t robot) const;

    ///
    /// \brief The moves cancelled by collisions in the current episode
    ///
    uint_t n_collisions() const noexcept { return n_collisions_; }

    ///
    /// \brief The smallest distance between the centres of two robots in the current episode
    ///
    real_t min_separation() const noexcept { return min_separation_; }

  private:
    time_step_type make_time_step_(TimeStepTp type, std::vector<real_t> rewards, bool truncated,
                                   uint_t collisions) const;

    std::vector<real_t> observe_(uint_t robot) const;

    real_t closest_pair_() const;

    std::vector<RobotPose> poses_;
    std::vector<std::array<real_t, 2>> goals_;
    std::vector<std::array<real_t, 2>> velocities_;
    std::vector<bool> dones_;
    real_t circle_radius_{2.0};
    real_t robot_radius_{0.15};
    real_t goal_tolerance_{0.1};
    real_t dt_{0.1};
    real_t max_v_{0.5};
    real_t max_w_{2.0};
    real_t progress_weight_{1.0};
    real_t time_reward_{-0.01};
    real_t collision_reward_{-1.0};
    real_t goal_reward_{10.0};
    real_t start_noise_{0.0};
    uint_t n_collisions_{0};
    real_t min_separation_{0.0};
};

} // namespace envs::multi_agent
} // namespace bitrl

#endif // MULTI_ROBOT_NAVIGATION_H
