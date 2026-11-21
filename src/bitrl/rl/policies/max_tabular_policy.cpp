#include "bitrl/rl/policies/max_tabular_policy.h"
#include "bitrl/base/bitrl_config.h"
#include "bitsim/utils/io/csv_file_writer.h"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <algorithm>
#include <iterator>
#include <vector>

#ifdef BITRL_DEBUG
#include <cassert>
#endif

namespace bitrl
{
namespace rl
{
namespace policies
{

template <>
uint_t MaxTabularPolicy::get_action(const std::vector<std::vector<real_t>> &q_map, uint_t state_idx)
{

#ifdef BITRL_DEBUG
    assert(state_idx < q_map.size() && "Invalid state index. Should be state_idx < q_map.size()");
#endif

    return MaxTabularPolicy::get_action(q_map[state_idx]);
}

template <> uint_t MaxTabularPolicy::get_action(const DynMat<real_t> &mat, uint_t state_idx)
{

#ifdef BITRL_DEBUG
    assert(state_idx < mat.size() && "Invalid state index. Should be state_idx < q_map.size()");
#endif

    return MaxTabularPolicy::get_action(mat.row(state_idx));
}

void MaxTabularPolicy::save(const std::string &filename) const
{

    typedef MaxTabularPolicy::state_type state_type;
    typedef MaxTabularPolicy::action_type action_type;

    bitsim::utils::io::CSVWriter csv_writer(filename);
    csv_writer.open();

    csv_writer.write_column_names({"state", "action"});

    for (uint_t s = 0; s < state_action_map_.size(); ++s)
    {
        std::tuple<state_type, action_type> row = {s, state_action_map_[s]};
        csv_writer.write_row(row);
    }

    csv_writer.close();
}

void MaxTabularPolicy::load(const std::string &filename)
{
    std::ifstream file(filename);
    if (!file)
    {
        throw std::runtime_error("MaxTabularPolicy: cannot open " + filename);
    }

    std::vector<uint_t> state_action_map;
    std::string line;
    while (std::getline(file, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue;
        }
        std::istringstream row(line);
        uint_t state = 0;
        uint_t action = 0;
        char comma = 0;
        if (!(row >> state >> comma >> action) || comma != ',')
        {
            throw std::runtime_error("MaxTabularPolicy: " + filename + " has a bad row: " + line);
        }
        if (state != state_action_map.size())
        {
            throw std::runtime_error("MaxTabularPolicy: " + filename + " gives state " +
                                     std::to_string(state) + " where state " +
                                     std::to_string(state_action_map.size()) + " should be");
        }
        state_action_map.push_back(action);
    }
    state_action_map_ = std::move(state_action_map);
}

void MaxTabularPolicyBuilder::build_from_state_action_function(const DynMat<real_t> &q,
                                                               MaxTabularPolicy &policy)
{

    policy.state_action_map_.clear();
    policy.state_action_map_.resize(q.rows());

    for (uint_t s = 0; s < q.rows(); ++s)
    {
        auto action = policy.get_action(q, s);
        policy.state_action_map_[s] = action;
    }
}

} // namespace policies
} // namespace rl
} // namespace bitrl
