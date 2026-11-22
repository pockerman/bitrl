#include "bitrl/envs/multi_agent/multi_agent_grid_world.h"

#include <algorithm>
#include <cctype>
#include <map>
#include <stdexcept>

namespace bitrl
{
namespace envs::multi_agent
{

std::vector<std::string> MultiAgentGridWorld::default_layout()
{
    return {
        "#########", //
        "#A.###.B#", //
        "#.......#", //
        "#b.###.a#", //
        "#########", //
    };
}

MultiAgentGridWorld::MultiAgentGridWorld() : base_type("MultiAgentGridWorld") {}

void MultiAgentGridWorld::make(const std::string &version,
                               const std::unordered_map<std::string, std::any> &make_options,
                               const std::unordered_map<std::string, std::any> &reset_options)
{
    set_max_steps_(50);
    base_type::make(version, make_options, reset_options);

    layout_ = option_<std::vector<std::string>>("layout", default_layout());
    observation_ = option_<GridObservation>("observation", GridObservation::OWN_POSITION);
    shared_reward_ = option_<bool>("shared_reward", false);
    step_reward_ = option_<real_t>("step_reward", -0.05);
    goal_reward_ = option_<real_t>("goal_reward", 1.0);
    collision_reward_ = option_<real_t>("collision_reward", -0.5);

    if (layout_.empty() || layout_.front().empty())
    {
        throw std::invalid_argument("MultiAgentGridWorld: the layout is empty");
    }
    n_rows_ = layout_.size();
    n_cols_ = layout_.front().size();

    std::map<uint_t, cell_type> starts;
    std::map<uint_t, cell_type> goals;
    for (uint_t r = 0; r < n_rows_; ++r)
    {
        if (layout_[r].size() != n_cols_)
        {
            throw std::invalid_argument(
                "MultiAgentGridWorld: the rows of the layout differ in length");
        }
        for (uint_t c = 0; c < n_cols_; ++c)
        {
            const char ch = layout_[r][c];
            if (std::isupper(static_cast<unsigned char>(ch)))
            {
                starts[static_cast<uint_t>(ch - 'A')] = {r, c};
            }
            else if (std::islower(static_cast<unsigned char>(ch)))
            {
                goals[static_cast<uint_t>(ch - 'a')] = {r, c};
            }
            else if (ch != '#' && ch != '.')
            {
                throw std::invalid_argument(std::string("MultiAgentGridWorld: unknown cell '") +
                                            ch + "' in the layout");
            }
        }
    }

    if (starts.empty())
    {
        throw std::invalid_argument("MultiAgentGridWorld: the layout has no agents");
    }
    starts_.clear();
    goals_.clear();
    std::vector<std::string> names;
    for (uint_t a = 0; a < starts.size(); ++a)
    {
        if (!starts.contains(a) || !goals.contains(a))
        {
            throw std::invalid_argument(std::string("MultiAgentGridWorld: agent ") +
                                        static_cast<char>('A' + a) +
                                        " needs a start and a goal, and the letters must be "
                                        "consecutive from A");
        }
        starts_.push_back(starts[a]);
        goals_.push_back(goals[a]);
        names.push_back(std::string(1, static_cast<char>('A' + a)));
    }
    if (goals.size() != starts.size())
    {
        throw std::invalid_argument("MultiAgentGridWorld: a goal has no agent");
    }
    set_agents_(names);

    n_states_ = n_cells();
    if (observation_ == GridObservation::JOINT_POSITIONS)
    {
        for (uint_t a = 1; a < n_agents(); ++a)
        {
            n_states_ *= n_cells();
        }
    }

    positions_ = starts_;
    dones_.assign(n_agents(), false);
    make_created_();
}

bool MultiAgentGridWorld::is_free(const cell_type &cell) const noexcept
{
    return cell.first < n_rows_ && cell.second < n_cols_ && layout_[cell.first][cell.second] != '#';
}

uint_t MultiAgentGridWorld::observe_(uint_t agent) const
{
    uint_t observation = cell_index(positions_[agent]);
    if (observation_ == GridObservation::JOINT_POSITIONS)
    {
        uint_t scale = n_cells();
        for (uint_t other = 0; other < n_agents(); ++other)
        {
            if (other != agent)
            {
                observation += scale * cell_index(positions_[other]);
                scale *= n_cells();
            }
        }
    }
    return observation;
}

MultiAgentGridWorld::time_step_type
MultiAgentGridWorld::make_time_step_(TimeStepTp type, std::vector<real_t> rewards, bool truncated,
                                     uint_t collisions) const
{
    std::vector<uint_t> observations(n_agents());
    for (uint_t a = 0; a < n_agents(); ++a)
    {
        observations[a] = observe_(a);
    }
    return time_step_type(type, std::move(observations), std::move(rewards), dones_, truncated,
                          {{"collisions", std::any(collisions)}});
}

MultiAgentGridWorld::time_step_type MultiAgentGridWorld::reset(uint_t /*seed*/)
{
    if (!is_created())
    {
        throw std::logic_error("MultiAgentGridWorld: call make() before reset()");
    }
    // the starts are fixed, so the seed changes nothing
    positions_ = starts_;
    dones_.assign(n_agents(), false);
    n_collisions_ = 0;
    start_episode_();
    return make_time_step_(TimeStepTp::FIRST, std::vector<real_t>(n_agents(), 0.0), false, 0);
}

MultiAgentGridWorld::time_step_type MultiAgentGridWorld::step(const joint_action_type &actions)
{
    check_step_(actions);
    const uint_t n = n_agents();

    // where every agent wants to go; walls and done agents keep it in place
    std::vector<cell_type> targets = positions_;
    for (uint_t a = 0; a < n; ++a)
    {
        if (dones_[a])
        {
            continue;
        }
        if (actions[a] >= N_ACTIONS)
        {
            throw std::invalid_argument("MultiAgentGridWorld: action " +
                                        std::to_string(actions[a]) + " is not a move");
        }
        auto [r, c] = positions_[a];
        switch (static_cast<GridMove>(actions[a]))
        {
        case GridMove::UP:
            r = r == 0 ? r : r - 1;
            break;
        case GridMove::RIGHT:
            ++c;
            break;
        case GridMove::DOWN:
            ++r;
            break;
        case GridMove::LEFT:
            c = c == 0 ? c : c - 1;
            break;
        case GridMove::STAY:
            break;
        }
        if (is_free({r, c}))
        {
            targets[a] = {r, c};
        }
    }

    // cancel the moves that clash, until none does: cancelling one move can block another
    std::vector<bool> collided(n, false);
    bool changed = true;
    while (changed)
    {
        changed = false;
        for (uint_t a = 0; a < n; ++a)
        {
            for (uint_t b = a + 1; b < n; ++b)
            {
                const bool same_cell = targets[a] == targets[b];
                const bool swap = targets[a] == positions_[b] && targets[b] == positions_[a] &&
                                  targets[a] != positions_[a];
                if (!same_cell && !swap)
                {
                    continue;
                }
                // the agents that tried to move stay where they are; an agent that stays is
                // not pushed out of its cell, and is not to blame
                for (const uint_t k : {a, b})
                {
                    if (targets[k] != positions_[k])
                    {
                        targets[k] = positions_[k];
                        collided[k] = true;
                        changed = true;
                    }
                }
            }
        }
    }

    std::vector<real_t> rewards(n, 0.0);
    uint_t collisions = 0;
    for (uint_t a = 0; a < n; ++a)
    {
        if (dones_[a])
        {
            continue;
        }
        positions_[a] = targets[a];
        rewards[a] = step_reward_;
        if (collided[a])
        {
            rewards[a] += collision_reward_;
            ++collisions;
        }
        if (positions_[a] == goals_[a])
        {
            rewards[a] += goal_reward_;
            dones_[a] = true;
        }
    }
    n_collisions_ += collisions;

    if (shared_reward_)
    {
        real_t team = 0.0;
        for (const real_t r : rewards)
        {
            team += r;
        }
        std::fill(rewards.begin(), rewards.end(), team);
    }

    const bool all_done = std::all_of(dones_.begin(), dones_.end(), [](bool d) { return d; });
    const bool out_of_steps = count_step_();
    const bool last = all_done || out_of_steps;
    return make_time_step_(last ? TimeStepTp::LAST : TimeStepTp::MID, std::move(rewards),
                           out_of_steps && !all_done, collisions);
}

std::string MultiAgentGridWorld::render() const
{
    std::string out;
    for (uint_t r = 0; r < n_rows_; ++r)
    {
        std::string row = layout_[r];
        for (auto &ch : row)
        {
            // the starts are not part of the picture
            if (std::isupper(static_cast<unsigned char>(ch)))
            {
                ch = '.';
            }
        }
        for (uint_t a = 0; a < n_agents(); ++a)
        {
            if (positions_[a].first == r)
            {
                row[positions_[a].second] = static_cast<char>('A' + a);
            }
        }
        out += row + "\n";
    }
    return out;
}

} // namespace envs::multi_agent
} // namespace bitrl
