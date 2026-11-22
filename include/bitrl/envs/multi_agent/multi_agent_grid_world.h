#ifndef MULTI_AGENT_GRID_WORLD_H
#define MULTI_AGENT_GRID_WORLD_H

#include "bitrl/base/bitrl_types.h"
#include "bitrl/envs/multi_agent/multi_agent_env_base.h"

#include <any>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bitrl
{
namespace envs::multi_agent
{

///
/// \brief The moves of an agent of MultiAgentGridWorld
///
enum class GridMove : uint_t
{
    STAY = 0,
    UP = 1,
    RIGHT = 2,
    DOWN = 3,
    LEFT = 4
};

///
/// \brief What an agent of MultiAgentGridWorld observes
///
enum class GridObservation : uint_t
{
    OWN_POSITION = 0,   ///< its own cell only
    JOINT_POSITIONS = 1 ///< its own cell and the cells of the others
};

///
/// \brief Agents that walk a grid at the same time, each to its own goal, without bumping into
/// each other.
///
/// The map is a list of rows of characters: '#' is a wall, '.' a free cell, an upper case letter
/// the start of an agent and the same letter in lower case its goal; 'A' is agent 0, 'B' agent 1,
/// and so on. Every step each agent moves one cell up, right, down or left, or stays. A move into
/// a wall keeps the agent where it is. Moves that would put two agents in one cell, or make two
/// agents swap cells, are cancelled, and every agent whose move was cancelled gets
/// collision_reward and counts as a collision; an agent may follow another into the cell it
/// leaves. An agent that reaches its goal is done and stays there.
///
/// Every agent that is not done gets step_reward per step, and goal_reward when it arrives.
/// With shared_reward every agent gets the sum of the rewards of all, so that they cooperate.
///
/// An observation is a cell index, row * n_cols() + col, of the agent itself
/// (GridObservation::OWN_POSITION) or of all agents (GridObservation::JOINT_POSITIONS): own cell
/// + n_cells * (cell of the first other agent) + n_cells^2 * (cell of the second) and so on, so
/// that tabular methods can learn with either. n_states() is the number of observations.
///
/// Make options, all optional: "layout" (std::vector<std::string>, default default_layout()),
/// "observation" (GridObservation, default OWN_POSITION), "shared_reward" (bool, false),
/// "step_reward" (real_t, -0.05), "goal_reward" (real_t, 1.0), "collision_reward" (real_t, -0.5),
/// "max_steps" (uint_t, 50).
///
class MultiAgentGridWorld final : public MultiAgentEnvBase<uint_t, uint_t>
{
  public:
    typedef MultiAgentEnvBase<uint_t, uint_t> base_type;
    typedef typename base_type::time_step_type time_step_type;
    typedef typename base_type::joint_action_type joint_action_type;

    ///
    /// \brief A cell: row, column
    ///
    typedef std::pair<uint_t, uint_t> cell_type;

    static constexpr uint_t N_ACTIONS = 5;

    ///
    /// \brief Two rooms joined by a corridor one cell wide. Agent A goes from the top left to the
    /// bottom right, B from the top right to the bottom left, so they meet in the corridor and
    /// one of them has to let the other through.
    ///
    static std::vector<std::string> default_layout();

    MultiAgentGridWorld();

    void make(const std::string &version,
              const std::unordered_map<std::string, std::any> &make_options,
              const std::unordered_map<std::string, std::any> &reset_options) override;

    using base_type::reset;

    time_step_type reset(uint_t seed) override;

    time_step_type step(const joint_action_type &actions) override;

    uint_t n_rows() const noexcept { return n_rows_; }

    uint_t n_cols() const noexcept { return n_cols_; }

    uint_t n_cells() const noexcept { return n_rows_ * n_cols_; }

    ///
    /// \brief The number of observations of one agent
    ///
    uint_t n_states() const noexcept { return n_states_; }

    uint_t n_actions() const noexcept { return N_ACTIONS; }

    GridObservation observation_type() const noexcept { return observation_; }

    ///
    /// \brief Whether a cell is inside the map and not a wall
    ///
    bool is_free(const cell_type &cell) const noexcept;

    ///
    /// \brief The index of a cell, row * n_cols() + col
    ///
    uint_t cell_index(const cell_type &cell) const noexcept
    {
        return cell.first * n_cols_ + cell.second;
    }

    const std::vector<cell_type> &positions() const noexcept { return positions_; }

    const std::vector<cell_type> &goals() const noexcept { return goals_; }

    ///
    /// \brief The moves cancelled by collisions in the current episode
    ///
    uint_t n_collisions() const noexcept { return n_collisions_; }

    ///
    /// \brief The map with the agents in upper case and their goals in lower case
    ///
    std::string render() const;

  private:
    time_step_type make_time_step_(TimeStepTp type, std::vector<real_t> rewards, bool truncated,
                                   uint_t collisions) const;

    uint_t observe_(uint_t agent) const;

    std::vector<std::string> layout_;
    std::vector<cell_type> starts_;
    std::vector<cell_type> goals_;
    std::vector<cell_type> positions_;
    std::vector<bool> dones_;
    uint_t n_rows_{0};
    uint_t n_cols_{0};
    uint_t n_states_{0};
    uint_t n_collisions_{0};
    GridObservation observation_{GridObservation::OWN_POSITION};
    bool shared_reward_{false};
    real_t step_reward_{-0.05};
    real_t goal_reward_{1.0};
    real_t collision_reward_{-0.5};
};

} // namespace envs::multi_agent
} // namespace bitrl

#endif // MULTI_AGENT_GRID_WORLD_H
